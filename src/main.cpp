// 文件路径: ~/smart_terminal/src/main.cpp
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <sys/time.h>
#include <mosquitto.h> // 【新增】MQTT 头文件

// === DRM KMS 现代显示框架 ===
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h> // 使用你 SDK 适配的路径

// === RKNN + RGA ===
#include "yolov5.h"
#include "postprocess.h"
#include "image_drawing.h"
#include "image_utils.h"
#include "im2d.h"
#include "RgaApi.h"

#define CAM_DEV "/dev/video0"
#define CAM_WIDTH  1280
#define CAM_HEIGHT 720
#define REQ_BUF_CNT 4

#define MODEL_WIDTH  640
#define MODEL_HEIGHT 640

#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 800

#include <mosquitto.h> // 【新增】MQTT 头文件


//====================================================================

// ==========================================
// 【新增】：MQTT 配置参数
// ==========================================
// === MQTT 配置参数 ===
#define MQTT_HOST       "z9c1fa31.ala.cn-hangzhou.emqxsl.cn"
#define MQTT_PORT       8883
#define MQTT_USER       "rk3566dev"
#define MQTT_PWD        "2cpvhZLTX8UfX4R"
#define CA_CERT_PATH    "./emqxsl-ca.crt"  // CA证书路径
#define MQTT_TOPIC      "rk3566/test_topic"

// 连接成功后的回调函数
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        printf("[SUCCESS] 成功通过 TLS 连接到 EMQX Serverless Broker!\n");
        
        // 1. 订阅测试主题
        mosquitto_subscribe(mosq, NULL, MQTT_TOPIC, 1);
        printf("[INFO] 已发送订阅请求: %s\n", MQTT_TOPIC);
        
        // 2. 发布一条测试消息
        const char *msg = "Hello from RK3566 Standalone MQTT Test!";
        mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(msg), msg, 1, false);
        printf("[INFO] 已发送测试消息: %s\n", msg);
    } else {
        printf("[ERROR] 连接失败，错误码: %d\n", rc);
    }
}

// 接收到消息的回调函数
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    printf("\n>>> [MQTT 接收消息] <<<\n");
    printf("主题: %s\n", msg->topic);
    printf("负载: %s\n", (char *)msg->payload);
    printf("QoS:  %d\n", msg->qos);
    printf("------------------------\n");
}
//====================================================================

// V4L2 缓冲区结构体，带有 dma_fd
struct CamBuffer { 
    void* start; 
    size_t length; 
    int dma_fd;  // 用于保存内核分配的物理内存 FD
};
CamBuffer* buffers = NULL;
int cam_fd = -1;

// DRM 全局变量
int drm_fd = -1;
uint32_t fb_id = 0;
uint32_t gem_handle = 0;
uint32_t pitch = 0;
void* fb_ptr = NULL;
uint64_t fb_size = 0;
drmModeCrtc *saved_crtc = NULL;
int screen_dma_fd = -1; // 显存的物理内存 FD



// ==========================================
// 【新增】：极致零拷贝的 DMA 内存结构体与分配器
// ==========================================
struct DmaBuffer {
    void* virt_addr; // CPU 画框用的虚拟地址
    uint64_t size;   // 内存大小
    uint32_t handle; // DRM 内部句柄
    int dma_fd;      // 喂给 RGA 的终极物理通行证
};

// 核心工具：向显卡驱动“白嫖”一块物理连续、无 Cache 污染的内存
int alloc_drm_dma_buffer(int drm_fd, uint32_t width, uint32_t height, uint32_t bpp, DmaBuffer* buf) 
{
    struct drm_mode_create_dumb create = {0};
    create.width = width;
    create.height = height;
    create.bpp = bpp; // 比如 RGB888 算作 24，但为对齐通常传 24 或 32
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
        perror("分配 DMA 内存失败");
        return -1;
    }

    buf->size = create.size;
    buf->handle = create.handle;

    struct drm_mode_map_dumb map = {0};
    map.handle = buf->handle;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) {
        perror("获取 DMA 映射偏移量失败");
        return -1;
    }

    // 映射给 CPU 画框用
    buf->virt_addr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
    if (buf->virt_addr == MAP_FAILED) {
        perror("DMA 内存 mmap 失败");
        return -1;
    }

    // 导出给 RGA 硬件加速用
    if (drmPrimeHandleToFD(drm_fd, buf->handle, DRM_CLOEXEC, &buf->dma_fd) < 0) {
        perror("导出 DMA_FD 失败");
        return -1;
    }

    memset(buf->virt_addr, 0, buf->size); // 清空脏数据
    return 0;
}

// 清理工具
void free_drm_dma_buffer(int drm_fd, DmaBuffer* buf) {
    if (buf->dma_fd > 0) close(buf->dma_fd);
    if (buf->virt_addr && buf->virt_addr != MAP_FAILED) munmap(buf->virt_addr, buf->size);
    if (buf->handle > 0) {
        struct drm_mode_destroy_dumb destroy = {0};
        destroy.handle = buf->handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
}


int main(int argc, char **argv)
{
    struct mosquitto *mosq = NULL;
    int rc;

    // 1. 初始化 mosquitto 库
    mosquitto_lib_init();

    // 2. 创建客户端实例
    mosq = mosquitto_new("rk3566_standalone_test", true, NULL);
    if (!mosq) {
        fprintf(stderr, "[ERROR] 无法创建 mosquitto 实例\n");
        return -1;
    }

    // 3. 注册回调函数
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    // 4. 配置用户名和密码
    mosquitto_username_pw_set(mosq, MQTT_USER, MQTT_PWD);

    // 5. 配置 TLS 证书 (必须在 connect 之前调用)
    // 参数依次为：CA文件, CA目录, 客户端证书, 客户端私钥, 密码回调
    rc = mosquitto_tls_set(mosq, CA_CERT_PATH, NULL, NULL, NULL, NULL);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[ERROR] TLS 证书配置失败: %s (请检查 %s 是否存在)\n", mosquitto_strerror(rc), CA_CERT_PATH);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return -1;
    }

    // 6. 连接到 Broker
    printf("[INFO] 正在连接到 %s:%d...\n", MQTT_HOST, MQTT_PORT);
    rc = mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[ERROR] 建立连接失败: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return -1;
    }

    // 7. 进入阻塞式事件循环 (按 Ctrl+C 退出)
    printf("[INFO] 进入 MQTT 事件循环，等待消息收发 (Ctrl+C 退出)...\n");
    mosquitto_loop_forever(mosq, -1, 1);

    // 8. 清理资源
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;



}



int fake_main(int argc, char **argv)
{
    if (argc != 2) { 
        printf("用法: %s <yolov5.rknn>\n", argv[0]); 
        return -1; 
    }
    init_post_process();
    
    // ==========================================
    // 1. 初始化 YOLOv5 大脑
    // ==========================================
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    if (init_yolov5_model(argv[1], &rknn_app_ctx) != 0) return -1;
    printf("[1] YOLOv5 模型加载成功！\n");

    // ==========================================
    // 2. DRM Dumb Buffer 初始化
    // ==========================================
    drm_fd = open("/dev/dri/card0", O_RDWR);
    if (drm_fd < 0) { perror("打开 /dev/dri/card0 失败"); return -1; }

    drmModeRes *res = drmModeGetResources(drm_fd);
    drmModeConnector *conn = NULL;
    for (int i = 0; i < res->count_connectors; ++i) {
        conn = drmModeGetConnector(drm_fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED) break;
        drmModeFreeConnector(conn);
        conn = NULL;
    }

    uint32_t crtc_id = res->crtcs[0];  
    drmModeEncoder *enc = drmModeGetEncoder(drm_fd, conn->encoders[0]);
    if (enc) {
        crtc_id = enc->crtc_id;
        drmModeFreeEncoder(enc);
    }
    saved_crtc = drmModeGetCrtc(drm_fd, crtc_id);

    struct drm_mode_create_dumb create = {0};
    create.width = SCREEN_WIDTH;
    create.height = SCREEN_HEIGHT;
    create.bpp = 32;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create);
    gem_handle = create.handle;
    pitch = create.pitch;
    fb_size = create.size;

    struct drm_mode_map_dumb map = {0};
    map.handle = gem_handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map);
    fb_ptr = mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
    
    //fb_ptr指向frame buffer 
    memset(fb_ptr, 0, fb_size); 

    drmModeAddFB(drm_fd, SCREEN_WIDTH, SCREEN_HEIGHT, 24, 32, pitch, gem_handle, &fb_id);
    
    drmModeSetCrtc(drm_fd, crtc_id, fb_id, 0, 0, &conn->connector_id, 1, &conn->modes[0]);
    
    // 将 DRM 的显存句柄，导出为 RGA 最喜欢的 DMA FD
    if (drmPrimeHandleToFD(drm_fd, gem_handle, DRM_CLOEXEC, &screen_dma_fd) == 0) {
        printf("[2] DRM 初始化成功，屏幕 DMA_FD: %d\n", screen_dma_fd);
    } else {
        perror("导出屏幕 DMA_FD 失败");
    }

    // ==========================================
    // 3. 初始化 V4L2 摄像头 (MPLANE 架构)
    // ==========================================
    cam_fd = open(CAM_DEV, O_RDWR);
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = CAM_WIDTH;
    fmt.fmt.pix_mp.height = CAM_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12; 
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    ioctl(cam_fd, VIDIOC_S_FMT, &fmt);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = REQ_BUF_CNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_REQBUFS, &req);

    buffers = (CamBuffer*)calloc(req.count, sizeof(CamBuffer));
    for (int i = 0; i < req.count; ++i) {
        struct v4l2_plane planes[1];
        memset(planes, 0, sizeof(planes));
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;

        ioctl(cam_fd, VIDIOC_QUERYBUF, &buf);
        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buf.m.planes[0].m.mem_offset);
        
        // 命令 V4L2 驱动，将物理内存导出为 DMA FD
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf, 0, sizeof(expbuf));
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        expbuf.index = i;
        expbuf.plane = 0;
        if (ioctl(cam_fd, VIDIOC_EXPBUF, &expbuf) == 0) {
            buffers[i].dma_fd = expbuf.fd;
            printf("  V4L2 成功导出 %d 号缓冲区 DMA_FD: %d\n", i, expbuf.fd);
        } else {
            perror("  V4L2 导出 DMA_FD 失败");
        }

        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(cam_fd, VIDIOC_STREAMON, &type);
    printf("[3] 摄像头 V4L2 流水线构建完毕！\n");

    // ==========================================
    // 4. 中转内存池 (给 CPU 画框用)
    // ==========================================
    DmaBuffer model_dma = {0};
    DmaBuffer canvas_dma = {0};

    // 为 YOLOv5 模型分配 640x640 RGB888 (bpp=24) 的连续物理内存
    if (alloc_drm_dma_buffer(drm_fd, MODEL_WIDTH, MODEL_HEIGHT, 24, &model_dma) < 0) return -1;
    
    // 为 1280x720 的高清画布分配 RGB888 的连续物理内存
    if (alloc_drm_dma_buffer(drm_fd, CAM_WIDTH, CAM_HEIGHT, 24, &canvas_dma) < 0) return -1;
    
    printf("[4] DMA 中转内存池分配成功！模型 FD: %d, 画布 FD: %d\n", model_dma.dma_fd, canvas_dma.dma_fd);

// 【核心修复】：彻底清空栈内存的垃圾值！
    image_buffer_t rknn_img;
    memset(&rknn_img, 0, sizeof(image_buffer_t)); 
    rknn_img.width = MODEL_WIDTH;
    rknn_img.height = MODEL_HEIGHT;
    rknn_img.width_stride = MODEL_WIDTH;   // 补齐官方要求的跨距
    rknn_img.height_stride = MODEL_HEIGHT; // 补齐官方要求的跨距
    rknn_img.format = IMAGE_FORMAT_RGB888;
    rknn_img.virt_addr = (unsigned char*)model_dma.virt_addr; 
    rknn_img.fd = model_dma.dma_fd;        // 【核心升华】：把真正的 DMA FD 交给官方库！

    // 【核心修复】：彻底清空画布的垃圾值！
    image_buffer_t canvas_img;
    memset(&canvas_img, 0, sizeof(image_buffer_t));
    canvas_img.width = CAM_WIDTH;
    canvas_img.height = CAM_HEIGHT;
    canvas_img.width_stride = CAM_WIDTH;
    canvas_img.height_stride = CAM_HEIGHT;
    canvas_img.format = IMAGE_FORMAT_RGB888;
    canvas_img.virt_addr = (unsigned char*)canvas_dma.virt_addr; 
    canvas_img.fd = canvas_dma.dma_fd;     // 【核心升华】

    // ==========================================
    // 5. 终极主循环 (极速 dma_fd 直通模式)
    // ==========================================
    printf("--- 开始 AI 视觉推理循环 (极速 dma_fd 直通模式) ---\n");
    // 【新增】：FPS 监测变量
    struct timeval start_time, end_time;
    int frame_count = 0;
    gettimeofday(&start_time, NULL);


    for (int frame = 0; frame < 500; ++frame) {
        struct v4l2_plane planes[1];
        memset(planes, 0, sizeof(planes));
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(cam_fd, VIDIOC_DQBUF, &buf) < 0) break;

        // 【RGA 处理 1】：喂给模型的缩放 (源和目的全都是 dma_fd！)
        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(buffers[buf.index].dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        
        // 核心修改：模型缓冲也用 dma_fd！
        rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
        imresize(rga_cam_nv12, rga_model_rgb); 

        // 【AI 推理】
        object_detect_result_list od_results;
        inference_yolov5_model(&rknn_app_ctx, &rknn_img, &od_results);

        // 【RGA 处理 2】：生成画布
        // 核心修改：画布缓冲也用 dma_fd！
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
        imcopy(rga_cam_nv12, rga_canvas_rgb);


        
        char text[256]; // 用于存放组合好的文字标签
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result* det = &od_results.results[i];
            
            // 坐标系还原：将 640x640 的推理坐标，放大回 1280x720 的原始画布坐标
            int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
            int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
            int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            
            // 终端打印调试信息（可选，如果觉得太刷屏可以注释掉）
            printf("检测到: %s, 概率: %.3f, 坐标: [%d, %d, %d, %d]\n", 
                   coco_cls_to_name(det->cls_id), det->prop, x1, y1, x1+w, y1+h);

            // 1. 画检测框 (参考官方，改成蓝色框，线宽 3)
            draw_rectangle(&canvas_img, x1, y1, w, h, COLOR_BLUE, 3);

            // 2. 格式化文字：类别 + 概率百分比
            sprintf(text, "%s %.1f%%", coco_cls_to_name(det->cls_id), det->prop * 100);
            
            // 3. 画文字 (红色字，字号 10)
            // 【安全保护】：如果框太靠上 (y1 < 20)，把文字画在框里面，防止写到内存外面导致段错误！
            int text_y = (y1 - 20 < 10) ? (y1 + 10) : (y1 - 20); 
            draw_text(&canvas_img, text, x1, text_y, COLOR_RED, 10);
        }

        // 【RGA 处理 3】：目标屏幕同样使用 dma_fd 接收 RGA 数据！
        // 因为我们在 DRM 里申请了 32 bpp，所以格式绝对是 RK_FORMAT_BGRA_8888，不用再判断了！
        int fb_format = RK_FORMAT_BGRA_8888;
        rga_buffer_t dst_fb = wrapbuffer_fd(screen_dma_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        
        // RGA 旋转并直接操作 DRM 显存的物理地址 (零拷贝刷屏)
        // 将在画布中画的图像，通过rga实际写入到drm的显存中，即可实现刷新效果
        IM_STATUS status = imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        // 【新增】：如果 RGA 罢工，立刻在终端大声报警！
        if (status != IM_STATUS_SUCCESS) {
            printf("RGA 旋转失败! 错误码: %s\n", imStrError(status));
        }
        ioctl(cam_fd, VIDIOC_QBUF, &buf);

        // 【新增】：每跑 30 帧，统计并打印一次平均 FPS
        frame_count++;
        if (frame_count % 30 == 0) {
            gettimeofday(&end_time, NULL);
            // 计算经过了多少秒
            float elapsed = (end_time.tv_sec - start_time.tv_sec) + 
                            (end_time.tv_usec - start_time.tv_usec) / 1000000.0f;
            // 计算 FPS (30帧 / 耗时)
            float fps = 30.0f / elapsed;
            
            printf("当前全链路平均 FPS: %.2f \n", fps);
            
            // 重置计时器，准备统计下一个 30 帧
            gettimeofday(&start_time, NULL); 
        }
    }

    // ==========================================
    // 6. 清理战场 (释放资源)
    // ==========================================
    printf("--- 抓图结束，开始清理资源 ---\n");
    
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < REQ_BUF_CNT; ++i) {
        if (buffers[i].dma_fd > 0) close(buffers[i].dma_fd); // 必须关闭导出的 FD
        if (buffers[i].start != NULL && buffers[i].start != MAP_FAILED) {
            munmap(buffers[i].start, buffers[i].length);
        }
    }
    free(buffers);
    struct v4l2_requestbuffers req_free;
    memset(&req_free, 0, sizeof(req_free));
    req_free.count = 0; 
    req_free.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req_free.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_REQBUFS, &req_free);

// 释放我们自己造的 DMA 内存池
    free_drm_dma_buffer(drm_fd, &model_dma);
    free_drm_dma_buffer(drm_fd, &canvas_dma);
    
    if (saved_crtc) {
        drmModeSetCrtc(drm_fd, saved_crtc->crtc_id, saved_crtc->buffer_id,
                       saved_crtc->x, saved_crtc->y, &conn->connector_id, 1, &saved_crtc->mode);
        drmModeFreeCrtc(saved_crtc);
    }
    drmModeRmFB(drm_fd, fb_id);
    struct drm_mode_destroy_dumb destroy = {0};
    destroy.handle = gem_handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    munmap(fb_ptr, fb_size);
    if(screen_dma_fd > 0) close(screen_dma_fd);
    
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
    close(drm_fd);
    close(cam_fd);
    release_yolov5_model(&rknn_app_ctx);
    
    printf("程序安全退出！\n");
    return 0;
}