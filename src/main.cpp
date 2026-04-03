
/**
 * @file main.cpp   
 * @author kikyou
 * @brief 
 * @version 0.1
 * @date 2026-04-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h> 
#include <pthread.h>    // 引入多线程

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
// AI 异步线程的共享状态与互斥锁
// ================================================================
pthread_mutex_t g_ai_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  g_ai_cond  = PTHREAD_COND_INITIALIZER;

bool g_ai_thread_run = true;
bool g_ai_busy       = false;       // AI 是否正在推理
bool g_has_new_frame = false;       // 是否有新画面需要推理
bool g_ai_wants_snap = false;       // 传递给 AI 线程的强制抓拍标志

// AI 计算完毕后输出的最新结果（供主线程渲染使用）
object_detect_result_list g_display_results = {0};
bool g_trigger_snapshot = false;    // 触发主线程去抓拍落盘
bool g_trigger_is_force = false;    // 是否是强制抓拍触发
int  g_trigger_person_count = 0;    // 检测到的人数

// 传递给 AI 线程的参数结构体
struct AI_Thread_Args {
    rknn_app_context_t* ctx;
    image_buffer_t* img;            //要推理的图像数据
};

// ================================================================
// AI 独立推理后台线程
// ================================================================
void* ai_worker_thread(void* arg) {
    AI_Thread_Args* args = (AI_Thread_Args*)arg;

    printf("[AI Worker] ai inference thread running...\n");



    while(g_ai_thread_run) {
        // 1. 阻塞等待主线程“摇铃铛”
        pthread_mutex_lock(&g_ai_mutex);
        while(!g_has_new_frame && g_ai_thread_run) {
            pthread_cond_wait(&g_ai_cond, &g_ai_mutex);
        }
        g_has_new_frame = false;
        g_ai_busy = true;            // 声明自己正在忙，主线程不要再喂图了
        bool is_force_snap = g_ai_wants_snap;
        g_ai_wants_snap = false;
        pthread_mutex_unlock(&g_ai_mutex);

        if (!g_ai_thread_run) break;

        // 2. 执行耗时的 NPU 推理
        object_detect_result_list local_results;
        inference_yolov5_model(args->ctx, args->img, &local_results);

        // 3. 过滤结果，cls_id == 0 是人，且置信度要大于云端设定的阈值
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

/**
 * @brief 云端 MQTT 指令处理回调函数，支持动态调整 AI 参数和触发强制抓拍
 * 实现对于设备的反向控制
 */
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
        printf("\n[Main] receive (Signo: %d),begin clean up resources.....\n", signo);
        g_quit_flag = 1;
    }
}
// 使用 RGA 硬件绘制空心矩形框
static void rga_draw_rectangle(rga_buffer_t canvas, int rx, int ry, int rw, int rh, int color, int thickness)
{
    // 防止越界
    if (rx < 0) rx = 0;
    if (ry < 0) ry = 0;
    
    // 1. 顶部横线
    im_rect rect_top = {rx, ry, rw, thickness};
    imfill(canvas, rect_top, color);

    // 2. 底部横线
    im_rect rect_bottom = {rx, ry + rh - thickness, rw, thickness};
    imfill(canvas, rect_bottom, color);

    // 3. 左侧竖线 (注意高度减去上下两条横线的厚度，防止重复绘制)
    im_rect rect_left = {rx, ry + thickness, thickness, rh - 2 * thickness};
    imfill(canvas, rect_left, color);

    // 4. 右侧竖线
    im_rect rect_right = {rx + rw - thickness, ry + thickness, thickness, rh - 2 * thickness};
    imfill(canvas, rect_right, color);
}



/**
 * @brief 入口
 * 
 * @param argc  理论上接收两个参数
 * @param argv  argv[1]存放yolo5.rknn模型文件的路径
 * @return int 
 */
int main(int argc, char **argv)
{
    if (argc != 2) { printf("usage: %s <yolov5.rknn_path>\n", argv[0]); return -1; }

    printf("\n =====================begin to run================== \n");

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    mqtt_set_cmd_callback(cloud_cmd_handler);

    if (mqtt_init() < 0) printf("[Error] MQTT init fail!\n");
    if (hal_sensor_init() < 0) printf("[Error] Sensor init!\n");
    if (service_storage_init() < 0) printf("[Error] Storage Worker init!\n");
    if (hal_display_init() < 0) 
    {
        printf("[Error] DRM init fail\n");
        return -1;   
    }
    if (hal_camera_init() < 0)
    {
        printf("[Error] Camera init fail\n");
        return -1;
    } 

    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    if (init_yolov5_model(argv[1], &rknn_app_ctx) != 0) return -1;

    hal_drm_buf_t model_dma = {0};      //存放模型输入Image的DMA buffer
    hal_drm_buf_t canvas_dma = {0};     //存放CPU画框用的Canvas buffer
    hal_drm_buf_t publish_dma = {0};    //存放要发布的图片数据的DMA buffer

    //分配yolov5推理输入的图片数据缓冲区
    hal_display_alloc_buffer(MODEL_WIDTH, MODEL_HEIGHT, 24, &model_dma);
    
    //分配供cpu画框用的canvas缓冲区
    hal_display_alloc_buffer(CAM_WIDTH, CAM_HEIGHT, 24, &canvas_dma);
    
    //分配供publish用的缓冲区
    hal_display_alloc_buffer(CAM_WIDTH, CAM_HEIGHT, 24, &publish_dma);
    
    //封装为rknn模型输入格式的buffer
    image_buffer_t rknn_img = {0};
    rknn_img.width = MODEL_WIDTH; rknn_img.height = MODEL_HEIGHT; rknn_img.width_stride = MODEL_WIDTH; 
    rknn_img.height_stride = MODEL_HEIGHT;
    rknn_img.format = IMAGE_FORMAT_RGB888; rknn_img.virt_addr = (unsigned char*)model_dma.virt_addr;
    rknn_img.fd = model_dma.dma_fd;


    // 启动 AI 后台推理线程
    AI_Thread_Args ai_args = {&rknn_app_ctx, &rknn_img};
    pthread_t ai_tid;
    pthread_create(&ai_tid, NULL, ai_worker_thread, &ai_args);

    printf("---main thread begin run---\n");
    struct timeval last_publish_time = {0}; 

    while (!g_quit_flag) {
        
        int cam_dma_fd = -1;
        int cam_buf_index = -1;
        
        // 1.获得一帧，索引以及dma file descriptor
        if (hal_camera_get_frame(&cam_dma_fd, &cam_buf_index) < 0) break;

        //封装为RGA使用的格式
        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(cam_dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);

        // RGA 硬件直接从摄像头的 DMA buffer拷贝并转换格式到画框用的 Canvas，零 CPU 占用
        imcopy(rga_cam_nv12, rga_canvas_rgb); 
        
        current_sensor_state_t sensor_state = hal_sensor_get_state();
        //是否需要运行ai 推理
        bool run_ai = sensor_state.is_ai_active || g_force_snapshot;
        
        // 2. 将画面“喂”给 AI 线程 (极速硬件缩放，零内存拷贝)
        pthread_mutex_lock(&g_ai_mutex);
        //当ai当前没在推理的时候，
        if (!g_ai_busy && run_ai) {
            rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
            
            //将摄像头这一帧转换为模型所需要的格式和尺寸
            imresize(rga_cam_nv12, rga_model_rgb); // RGA 硬件瞬间完成
            
            //如果说client发送了强制抓拍上传的命令
            if (g_force_snapshot) {
                g_ai_wants_snap = true;
                g_force_snapshot = false; // 消费掉云端指令
            }
            
            g_has_new_frame = true;
            pthread_cond_signal(&g_ai_cond); // 摇铃铛唤醒 AI 线程
        }
        pthread_mutex_unlock(&g_ai_mutex);

        // 3. 读取AI 计算结果（即使 AI 还在算上一帧，这里也能瞬间拿到上上次的结果）
        // 注意：这里的结果是上一次 AI 线程计算的结果，可能会有一帧的延迟，但绝对不会卡顿
        pthread_mutex_lock(&g_ai_mutex);
        object_detect_result_list draw_results = g_display_results;
        bool do_snap = g_trigger_snapshot;
        bool is_force = g_trigger_is_force;
        int snap_count = g_trigger_person_count;
        g_trigger_snapshot = false; // 清除触发标志
        pthread_mutex_unlock(&g_ai_mutex);

        // 4. 这边仅仅绘制目标边界框，不再绘制类别名文本，由于绘制类别名需要调用软件绘制接口，
        //性能较差，纯cpu绘制导致延迟严重，画面卡顿
        char text[256]; 
        for (int i = 0; i < draw_results.count; i++) {
            object_detect_result* det = &draw_results.results[i];
            int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
            int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
            int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));

            //这边使用硬件RGA画框，完全不占用CPU，不会受到检测框数量增多而导致延迟显著加大
            rga_draw_rectangle(rga_canvas_rgb, x1, y1, w, h, 0xFF0000FF, 3);
        }

        // 5. 强制抓拍上传
        if (do_snap) {
            //检测到人了
            if (snap_count > 0) hal_sensor_keep_ai_alive(hal_sensor_get_watchdog_duration());

            struct timeval current_time;
            gettimeofday(&current_time, NULL);
            float time_since_last_pub = (current_time.tv_sec - last_publish_time.tv_sec) + 
                                        (current_time.tv_usec - last_publish_time.tv_usec) / 1000000.0f;

            //距离上次上传时间已经过了3秒，再上传，免得太过于频繁上传
            if (time_since_last_pub > 3.0f || is_force) {
                 
                rga_buffer_t rga_publish = wrapbuffer_fd(publish_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
                //将画框后的图像拷贝到发布用的缓冲区，准备发给mqtt线程去发布，零CPU占用  
                imcopy(rga_canvas_rgb, rga_publish);
                
                    //把这张图连同环境数据一起发给存储线程，存储线程负责落盘和云端同步
                if (service_storage_push_task((unsigned char*)publish_dma.virt_addr, CAM_WIDTH, CAM_HEIGHT, 
                                                snap_count, sensor_state.temp, sensor_state.humi, 
                                                sensor_state.co2, sensor_state.tvoc) == 0) {
                    last_publish_time = current_time; 
                    if (is_force) printf("[Main] cloudy force snapshot finish,async publish mqtt broker...\n");
                    else printf("[Main] [EVENT] detect %d target, async publish mqtt broker...\n", snap_count);
                }
                    
            } 
        }

        // 6. RGA 刷屏 (配合 DRM 双缓冲，绝不撕裂)
        int fb_format = RK_FORMAT_BGRA_8888;
        int back_screen_fd = hal_display_get_back_buffer_fd();
        rga_buffer_t dst_fb = wrapbuffer_fd(back_screen_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        
        //RGA转换后直接写入DRM的后台缓冲，零拷贝
        imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        //再VBLANK时提交屏幕更新请求，显示新画面
        hal_display_commit_and_wait();

        // 7. 告诉摄像头这一帧处理完了，可以复用这个buffer了
        hal_camera_put_frame(cam_buf_index);


    }

    printf("\n---begin to freee resources---\n");
    
    // 安全停止 AI 线程
    g_ai_thread_run = false;
    pthread_mutex_lock(&g_ai_mutex);
    g_has_new_frame = true;             // 唤醒阻塞在 wait 上的线程让其自然退出
    pthread_cond_signal(&g_ai_cond);
    pthread_mutex_unlock(&g_ai_mutex);
    pthread_join(ai_tid, NULL);

    service_storage_deinit(); 
    hal_display_free_buffer(&model_dma);
    hal_display_free_buffer(&canvas_dma);
    hal_display_free_buffer(&publish_dma);

    release_yolov5_model(&rknn_app_ctx);
    hal_camera_deinit();
    hal_display_deinit();
    mqtt_cleanup();
    
    printf("[Main] all resources have been cleaned,exit!\n");
    return 0;
}