// 文件路径: ~/smart_terminal/src/main.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h> 

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
float g_conf_threshold = 0.50f;  // 默认置信度阈值 50%
bool  g_force_snapshot = false;  // 强制抓拍标志

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
        // 使用 system 调用阻塞清理 (因为是在 MQTT 线程中执行，不影响主视频线程)
        system("rm -rf " SAVE_IMG_DIR "/*");
        system("rm -f " SAVE_CSV_PATH);
        printf("[Cloud CMD] 清理完成，磁盘空间已释放！\n");
    }
}

void sig_handler(int signo) 
{
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

    // 在初始化 MQTT 前，注册指令回调函数
    mqtt_set_cmd_callback(cloud_cmd_handler);

    if (mqtt_init() < 0) printf("[Error] MQTT 初始化失败！\n");
    if (hal_sensor_init() < 0) printf("[Error] Sensor 初始化失败！\n");
    if (service_storage_init() < 0) printf("[Error] Storage Worker 初始化失败！\n");
    if (hal_display_init() < 0) return -1;
    if (hal_camera_init() < 0) return -1;

    // 【删除】：不再在此处获取静态的 screen_dma_fd

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

    printf("--- 开始 AI 视觉推理循环 (已接入云端反向控制与双缓冲防撕裂) ---\n");
    struct timeval last_publish_time = {0}; 

    while (!g_quit_flag) {
        int cam_dma_fd = -1;
        int cam_buf_index = -1;
        if (hal_camera_get_frame(&cam_dma_fd, &cam_buf_index) < 0) break;

        current_sensor_state_t sensor_state = hal_sensor_get_state();
        
        // 如果是强制抓拍，哪怕目前处于 SLEEP 状态，也要强制唤醒 AI 引擎跑这一帧！
        bool run_ai = sensor_state.is_ai_active || g_force_snapshot;

        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(cam_dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
        imcopy(rga_cam_nv12, rga_canvas_rgb); 

        object_detect_result_list od_results;
        od_results.count = 0; 

        if (run_ai) {
            rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
            imresize(rga_cam_nv12, rga_model_rgb); 
            inference_yolov5_model(&rknn_app_ctx, &rknn_img, &od_results);

            int person_count = 0;
            bool person_found = false;

            if (od_results.count > 0) {
                char text[256]; 
                for (int i = 0; i < od_results.count; i++) {
                    object_detect_result* det = &od_results.results[i];
                    
                    if (det->cls_id != 0) continue; 
                    
                    // 应用云端动态下发的置信度阈值过滤！
                    if (det->prop < g_conf_threshold) continue;
                    
                    person_found = true;
                    person_count++;

                    int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
                    int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
                    int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
                    int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));
                    draw_rectangle(&canvas_img, x1, y1, w, h, 0xFF0000FF, 3); 

                    sprintf(text, "Person %.1f%%", det->prop * 100); 
                    int text_y = (y1 - 20 < 10) ? (y1 + 10) : (y1 - 20); 
                    draw_text(&canvas_img, text, x1, text_y, 0xFFFF0000, 15); 
                }
            }

            // 如果发现人，或者收到云端强行抓拍指令，都执行推流
            if (person_found || g_force_snapshot) {
                
                // 只有发现人时才给看门狗续杯（应用动态看门狗时间）
                if (person_found) hal_sensor_keep_ai_alive(hal_sensor_get_watchdog_duration());

                struct timeval current_time;
                gettimeofday(&current_time, NULL);
                float time_since_last_pub = (current_time.tv_sec - last_publish_time.tv_sec) + 
                                            (current_time.tv_usec - last_publish_time.tv_usec) / 1000000.0f;

                // 如果是强制抓拍，无视 3 秒的冷却期限制！
                if (time_since_last_pub > 3.0f || g_force_snapshot) {
                    if (service_storage_push_task((unsigned char*)canvas_img.virt_addr, CAM_WIDTH, CAM_HEIGHT, 
                                                  person_count, sensor_state.temp, sensor_state.humi, 
                                                  sensor_state.co2, sensor_state.tvoc) == 0) {
                        last_publish_time = current_time; 
                        
                        if (g_force_snapshot) {
                            printf("[Main] 强制抓拍执行完毕，图片已推入云端！\n");
                            g_force_snapshot = false; // 清除抓拍标志
                        } else {
                            printf("[Main] [EVENT] 画面中发现 %d 个人，已异步上云...\n", person_count);
                        }
                    } else {
                        printf("[Main] 警告：后台队列已满，抛弃该帧！\n");
                        g_force_snapshot = false; 
                    }
                } 
            }
        }

        // ================================================================
        // 【核心修改】：应用 DRM 双缓冲防撕裂逻辑
        // ================================================================
        int fb_format = RK_FORMAT_BGRA_8888;
        
        // 1. 动态获取当前的 Back Buffer (后台画布)
        int back_screen_fd = hal_display_get_back_buffer_fd();
        rga_buffer_t dst_fb = wrapbuffer_fd(back_screen_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        
        // 2. RGA 偷偷在后台画布上疯狂画图 (旋转并缩放输出到屏幕分辨率)
        imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        // 3. 提交 DRM 翻页，并挂起等待 VSync (垂直同步) 信号！
        hal_display_commit_and_wait();

        // 4. 用完归还 V4L2 缓冲
        hal_camera_put_frame(cam_buf_index);
    }

    printf("\n--- 开始安全清理系统资源 ---\n");
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