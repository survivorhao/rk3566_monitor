// 文件路径: ~/smart_terminal/src/main.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// 业务与底层接口
#include "app_config.h"
#include "service_mqtt.h"
#include "service_storage.h"  // 【新增】存储后台服务
#include "hal_sensor.h"
#include "hal_camera.h"
#include "hal_display.h"

// 算法层接口
#include "yolov5.h"
#include "postprocess.h"
#include "image_drawing.h"
#include "im2d.h"
#include "RgaApi.h"

int main(int argc, char **argv)
{
    if (argc != 2) { printf("用法: %s <yolov5.rknn>\n", argv[0]); return -1; }

    // 1. 初始化全链路基础服务
    if (mqtt_init() < 0) printf("[Error] MQTT 初始化失败！\n");
    if (hal_sensor_init() < 0) printf("[Error] Sensor 初始化失败！\n");
    if (service_storage_init() < 0) printf("[Error] Storage Worker 初始化失败！\n");
    if (hal_display_init() < 0) return -1;
    if (hal_camera_init() < 0) return -1;

    int screen_dma_fd = hal_display_get_screen_fd();

    // 2. 初始化 AI 模型与后处理
    init_post_process();
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    if (init_yolov5_model(argv[1], &rknn_app_ctx) != 0) return -1;

    // 3. 申请 AI 通道专用的零拷贝 DMA 内存池
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

    // 4. 开启状态机主循环
    printf("--- 开始 AI 视觉推理循环 (基于事件唤醒，聚焦人体检测) ---\n");
    struct timeval last_publish_time = {0}; 

    for (int frame = 0; frame < 50000; ++frame) {
        int cam_dma_fd = -1;
        int cam_buf_index = -1;
        if (hal_camera_get_frame(&cam_dma_fd, &cam_buf_index) < 0) break;

        current_sensor_state_t sensor_state = hal_sensor_get_state();
        bool run_ai = sensor_state.is_ai_active;

        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(cam_dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
        imcopy(rga_cam_nv12, rga_canvas_rgb); 

        object_detect_result_list od_results;
        od_results.count = 0; 

        if (run_ai) {
            rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
            imresize(rga_cam_nv12, rga_model_rgb); 
            inference_yolov5_model(&rknn_app_ctx, &rknn_img, &od_results);

            // ================================================================
            // 【核心优化点】：执行人体检测过滤逻辑
            // ================================================================
            int person_count = 0;
            bool person_found = false;

            if (od_results.count > 0) {
                // 第一步：遍历检测结果，找出所有的“人（Person）”，并绘制红框
                char text[256]; 
                for (int i = 0; i < od_results.count; i++) {
                    object_detect_result* det = &od_results.results[i];
                    
                    // 【关键过滤】：cls_id != 0 的物体直接忽略（例如：椅子、猫、狗）
                    if (det->cls_id != 0) continue; 
                    
                    // 确认是人，置位并累加
                    person_found = true;
                    person_count++;

                    // 绘制检测框和标签 (仅绘制人的)
                    int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
                    int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
                    int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
                    int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));
                    draw_rectangle(&canvas_img, x1, y1, w, h, 0xFF0000FF, 3); // 蓝色人框

                    sprintf(text, "Person %.1f%%", det->prop * 100); // 简化标签为 "Person"
                    int text_y = (y1 - 20 < 10) ? (y1 + 10) : (y1 - 20); 
                    draw_text(&canvas_img, text, x1, text_y, 0xFFFF0000, 10); // 红色标签文字
                }
            }

            // 第二步：根据过滤后的结果决定是否触发存储/上云逻辑
            if (person_found) {
                // YOLO确认有人，反向给硬件看门狗“续杯”
                hal_sensor_keep_ai_alive(10000);

                struct timeval current_time;
                gettimeofday(&current_time, NULL);
                float time_since_last_pub = (current_time.tv_sec - last_publish_time.tv_sec) + 
                                            (current_time.tv_usec - last_publish_time.tv_usec) / 1000000.0f;

                if (time_since_last_pub > 3.0f) {
                    // 【关键修改点】：
                    // 1. 只有发现人时才调用 Push Task。
                    // 2. 传入的“目标数量”必须是 person_count。
                    if (service_storage_push_task((unsigned char*)canvas_img.virt_addr, CAM_WIDTH, CAM_HEIGHT, 
                                                  person_count, sensor_state.temp, sensor_state.humi, 
                                                  sensor_state.co2, sensor_state.tvoc) == 0) {
                        last_publish_time = current_time; 
                        printf("[Main] 🚨 [EVENT] 画面中发现 %d 个人，已异步送入后台队列 (JPG+CSV+MQTT)...\n", person_count);
                    } else {
                        printf("[Main] 警告：后台队列已满，主动丢弃当前帧图像！\n");
                    }
                } 
            }
        }

        char osd_text[256];
        sprintf(osd_text, "AI: %s | T:%.1fC H:%.1f%% CO2:%d", run_ai ? "ACTIVE" : "SLEEP", 
                sensor_state.temp, sensor_state.humi, sensor_state.co2);
        draw_text(&canvas_img, osd_text, 10, 30, run_ai ? 0xFFFF0000 : 0xFF00FF00, 2); 

        int fb_format = RK_FORMAT_BGRA_8888;
        rga_buffer_t dst_fb = wrapbuffer_fd(screen_dma_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        hal_camera_put_frame(cam_buf_index);
    }

    // 清理 MQTT 服务
    mqtt_cleanup();
    return 0;
}