// 文件路径: ~/smart_terminal/src/main.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h> 
#include <pthread.h> // 【新增】引入多线程

#include "app_config.h"
#include "service_mqtt.h"
#include "service_storage.h"
#include "hal_sensor.h"
#include "hal_camera.h"
#include "hal_display.h"

#include "yolov5.h"
#include "postprocess.h"
#include "image_drawing.h"
#include "im2d.h"
#include "RgaApi.h"

volatile sig_atomic_t g_quit_flag = 0;

// ================================================================
// 云端动态控制全局变量
// ================================================================
float g_conf_threshold = 0.50f;  
bool  g_force_snapshot = false;  

// ================================================================
// 【核心新增】：AI 异步线程的共享状态与互斥锁
// ================================================================
pthread_mutex_t g_ai_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_ai_cond  = PTHREAD_COND_INITIALIZER;

bool g_ai_thread_run = true;
bool g_ai_busy       = false; // AI 是否正在推理
bool g_has_new_frame = false; // 是否有新画面需要推理
bool g_ai_wants_snap = false; // 传递给 AI 线程的强制抓拍标志

// AI 计算完毕后输出的最新结果（供主线程渲染使用）
object_detect_result_list g_display_results = {0};
bool g_trigger_snapshot = false; // 触发主线程去抓拍落盘
bool g_trigger_is_force = false; // 是否是强制抓拍触发
int  g_trigger_person_count = 0;

// 传递给 AI 线程的参数结构体
struct AI_Thread_Args {
    rknn_app_context_t* ctx;
    image_buffer_t* img;
};

// ================================================================
// 【核心新增】：AI 独立推理后台线程
// ================================================================
void* ai_worker_thread(void* arg) {
    AI_Thread_Args* args = (AI_Thread_Args*)arg;

    printf("[AI Worker] 后台推理引擎启动，等待画面喂入...\n");

    while(g_ai_thread_run) {
        // 1. 阻塞等待主线程“摇铃铛”
        pthread_mutex_lock(&g_ai_mutex);
        while(!g_has_new_frame && g_ai_thread_run) {
            pthread_cond_wait(&g_ai_cond, &g_ai_mutex);
        }
        g_has_new_frame = false;
        g_ai_busy = true; // 声明自己正在忙，主线程不要再喂图了
        bool is_force_snap = g_ai_wants_snap;
        g_ai_wants_snap = false;
        pthread_mutex_unlock(&g_ai_mutex);

        if (!g_ai_thread_run) break;

        // 2. 执行耗时的 NPU 推理 (此时完全不阻塞屏幕刷新！)
        object_detect_result_list local_results;
        inference_yolov5_model(args->ctx, args->img, &local_results);

        // 3. 过滤结果
        object_detect_result_list filtered = {0};
        for(int i = 0; i < local_results.count; i++) {
            if (local_results.results[i].cls_id == 0 && local_results.results[i].prop >= g_conf_threshold) {
                filtered.results[filtered.count++] = local_results.results[i];
            }
        }

        // 4. 将结果同步给全局变量
        pthread_mutex_lock(&g_ai_mutex);
        g_display_results = filtered;
        
        // 如果发现人，或者收到强制抓拍指令，通知主线程去抓拍
        if (filtered.count > 0 || is_force_snap) {
            g_trigger_snapshot = true;
            g_trigger_person_count = filtered.count;
            g_trigger_is_force = is_force_snap;
        }
        g_ai_busy = false; // 算完了，宣布空闲
        pthread_mutex_unlock(&g_ai_mutex);
    }
    
    printf("[AI Worker] 后台推理引擎安全退出。\n");
    return NULL;
}

// 云端指令分发中心
void cloud_cmd_handler(const char* cmd, float val) {
    if (strcmp(cmd, "set_timeout") == 0) {
        hal_sensor_set_watchdog_duration((int)(val * 1000));
        printf("\n[Cloud CMD] 成功！AI 唤醒时长修改为: %.1f 秒\n", val);
    } else if (strcmp(cmd, "set_threshold") == 0) {
        g_conf_threshold = val;
        printf("\n[Cloud CMD] 成功！AI 目标置信度阈值修改为: %.2f\n", val);
    } else if (strcmp(cmd, "snapshot") == 0) {
        g_force_snapshot = true;
        printf("\n[Cloud CMD] 收到强行抓拍指令，系统即将执行快门！\n");
    } else if (strcmp(cmd, "clear_storage") == 0) {
        printf("\n[Cloud CMD] 正在清理本地 SD 卡存储与日志...\n");
        system("rm -rf " SAVE_IMG_DIR "/*");
        system("rm -f " SAVE_CSV_PATH);
        printf("[Cloud CMD] 清理完成，磁盘空间已释放！\n");
    }
}

void sig_handler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        printf("\n[Main] 接收到退出信号 (Signo: %d)，准备优雅退出主循环...\n", signo);
        g_quit_flag = 1;
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) { printf("用法: %s <yolov5.rknn>\n", argv[0]); return -1; }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    mqtt_set_cmd_callback(cloud_cmd_handler);

    if (mqtt_init() < 0) printf("[Error] MQTT 初始化失败！\n");
    if (hal_sensor_init() < 0) printf("[Error] Sensor 初始化失败！\n");
    if (service_storage_init() < 0) printf("[Error] Storage Worker 初始化失败！\n");
    if (hal_display_init() < 0) return -1;
    if (hal_camera_init() < 0) return -1;

    init_post_process();
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    if (init_yolov5_model(argv[1], &rknn_app_ctx) != 0) return -1;

    hal_drm_buf_t model_dma = {0}; 
    hal_drm_buf_t canvas_dma = {0};
    hal_display_alloc_buffer(MODEL_WIDTH, MODEL_HEIGHT, 24, &model_dma);
    hal_display_alloc_buffer(CAM_WIDTH, CAM_HEIGHT, 24, &canvas_dma);
    
    image_buffer_t rknn_img = {0};
    rknn_img.width = MODEL_WIDTH; rknn_img.height = MODEL_HEIGHT; rknn_img.width_stride = MODEL_WIDTH; rknn_img.height_stride = MODEL_HEIGHT;
    rknn_img.format = IMAGE_FORMAT_RGB888; rknn_img.virt_addr = (unsigned char*)model_dma.virt_addr; rknn_img.fd = model_dma.dma_fd;

    image_buffer_t canvas_img = {0};
    canvas_img.width = CAM_WIDTH; canvas_img.height = CAM_HEIGHT; canvas_img.width_stride = CAM_WIDTH; canvas_img.height_stride = CAM_HEIGHT;
    canvas_img.format = IMAGE_FORMAT_RGB888; canvas_img.virt_addr = (unsigned char*)canvas_dma.virt_addr; canvas_img.fd = canvas_dma.dma_fd;

    // 【新增】：启动 AI 后台推理线程
    AI_Thread_Args ai_args = {&rknn_app_ctx, &rknn_img};
    pthread_t ai_tid;
    pthread_create(&ai_tid, NULL, ai_worker_thread, &ai_args);

    printf("--- 开始 AI 视觉流水线 (双线程异步 + 双缓冲防撕裂) ---\n");
    struct timeval last_publish_time = {0}; 

    while (!g_quit_flag) {
        int cam_dma_fd = -1;
        int cam_buf_index = -1;
        if (hal_camera_get_frame(&cam_dma_fd, &cam_buf_index) < 0) break;

        current_sensor_state_t sensor_state = hal_sensor_get_state();
        bool run_ai = sensor_state.is_ai_active || g_force_snapshot;

        // 1. 获取画面拷贝到 Canvas
        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(cam_dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
        imcopy(rga_cam_nv12, rga_canvas_rgb); 

        // 2. 将画面“喂”给 AI 线程 (极速硬件缩放，零内存拷贝)
        pthread_mutex_lock(&g_ai_mutex);
        if (!g_ai_busy && run_ai) {
            rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
            imresize(rga_cam_nv12, rga_model_rgb); // RGA 硬件瞬间完成
            
            if (g_force_snapshot) {
                g_ai_wants_snap = true;
                g_force_snapshot = false; // 消费掉云端指令
            }
            
            g_has_new_frame = true;
            pthread_cond_signal(&g_ai_cond); // 摇铃铛唤醒 AI 线程
        }
        pthread_mutex_unlock(&g_ai_mutex);

        // 3. 读取最新的 AI 计算结果（即使 AI 还在算上一帧，这里也能瞬间拿到上上次的结果）
        pthread_mutex_lock(&g_ai_mutex);
        object_detect_result_list draw_results = g_display_results;
        bool do_snap = g_trigger_snapshot;
        bool is_force = g_trigger_is_force;
        int snap_count = g_trigger_person_count;
        g_trigger_snapshot = false; // 清除触发标志
        pthread_mutex_unlock(&g_ai_mutex);

        // 4. 绘制画框
        char text[256]; 
        for (int i = 0; i < draw_results.count; i++) {
            object_detect_result* det = &draw_results.results[i];
            int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
            int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
            int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            draw_rectangle(&canvas_img, x1, y1, w, h, 0xFF0000FF, 3); 

            sprintf(text, "Person %.1f%%", det->prop * 100); 
            int text_y = (y1 - 20 < 10) ? (y1 + 10) : (y1 - 20); 
            draw_text(&canvas_img, text, x1, text_y, 0xFFFF0000, 15); 
        }

        // 5. 抓拍落盘逻辑
        if (do_snap) {
            if (snap_count > 0) hal_sensor_keep_ai_alive(hal_sensor_get_watchdog_duration());

            struct timeval current_time;
            gettimeofday(&current_time, NULL);
            float time_since_last_pub = (current_time.tv_sec - last_publish_time.tv_sec) + 
                                        (current_time.tv_usec - last_publish_time.tv_usec) / 1000000.0f;

            if (time_since_last_pub > 3.0f || is_force) {
                if (service_storage_push_task((unsigned char*)canvas_img.virt_addr, CAM_WIDTH, CAM_HEIGHT, 
                                                snap_count, sensor_state.temp, sensor_state.humi, 
                                                sensor_state.co2, sensor_state.tvoc) == 0) {
                    last_publish_time = current_time; 
                    if (is_force) printf("[Main] 📸 强制抓拍执行完毕，图片已异步推入云端！\n");
                    else printf("[Main] 🚨 [EVENT] 画面中发现 %d 个人，已异步上云...\n", snap_count);
                }
            } 
        }

        // 6. OSD 数据渲染
        sprintf(text, "AI: %s | T:%.1fC H:%.1f%% CO2:%d", run_ai ? "ACTIVE" : "SLEEP", 
                sensor_state.temp, sensor_state.humi, sensor_state.co2);
        draw_text(&canvas_img, text, 10, 30, run_ai ? 0xFFFF0000 : 0xFF00FF00, 2); 

        // 7. RGA 刷屏 (配合 DRM 双缓冲，绝不撕裂)
        int fb_format = RK_FORMAT_BGRA_8888;
        int back_screen_fd = hal_display_get_back_buffer_fd();
        rga_buffer_t dst_fb = wrapbuffer_fd(back_screen_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        hal_display_commit_and_wait();
        hal_camera_put_frame(cam_buf_index);
    }

    printf("\n--- 开始安全清理系统资源 ---\n");
    
    // 安全停止 AI 线程
    g_ai_thread_run = false;
    pthread_mutex_lock(&g_ai_mutex);
    g_has_new_frame = true; // 唤醒阻塞在 wait 上的线程让其自然退出
    pthread_cond_signal(&g_ai_cond);
    pthread_mutex_unlock(&g_ai_mutex);
    pthread_join(ai_tid, NULL);

    service_storage_deinit(); 
    hal_display_free_buffer(&model_dma);
    hal_display_free_buffer(&canvas_dma);
    release_yolov5_model(&rknn_app_ctx);
    hal_camera_deinit();
    hal_display_deinit();
    mqtt_cleanup();
    
    printf("[Main] 所有资源释放完毕，终端已安全关闭。再见！\n");
    return 0;
}