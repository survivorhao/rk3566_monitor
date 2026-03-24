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
#include <time.h>

// === 新增：TurboJPEG 和 OpenSSL Base64 支持 ===
#include <turbojpeg.h>
#include <openssl/evp.h>

// === MQTT 头文件 ===
#include <mosquitto.h> 

// === DRM KMS 现代显示框架 ===
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h> 

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

// ==========================================
// 【配置】：MQTT 参数
// ==========================================
#define MQTT_HOST       "z9c1fa31.ala.cn-hangzhou.emqxsl.cn"
#define MQTT_PORT       8883
#define MQTT_USER       "rk3566dev"
#define MQTT_PWD        "2cpvhZLTX8UfX4R"
#define CA_CERT_PATH    "./emqxsl-ca.crt"
#define MQTT_TOPIC      "rk3566/ai_events" // 改个名字，专门传AI事件

struct mosquitto *mqtt_client = NULL;

// MQTT 连接成功的回调函数
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        printf("[SUCCESS] 成功通过 TLS 连接到 EMQX Serverless Broker!\n");
        //mosquitto_subscribe(mosq, NULL, MQTT_TOPIC, 1);
    } else {
        printf("[ERROR] 连接失败，错误码: %d\n", rc);
    }
}

// 接收到消息的回调函数
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    printf("\n>>> [MQTT 接收消息] 主题: %s | 负载: %s\n", msg->topic, (char *)msg->payload);
}

// ==========================================
// V4L2 缓冲区结构体与 DRM 相关全局变量
// ==========================================
struct CamBuffer { 
    void* start; 
    size_t length; 
    int dma_fd;  
};
CamBuffer* buffers = NULL;
int cam_fd = -1;

int drm_fd = -1;
uint32_t fb_id = 0;
uint32_t gem_handle = 0;
uint32_t pitch = 0;
void* fb_ptr = NULL;
uint64_t fb_size = 0;
drmModeCrtc *saved_crtc = NULL;
int screen_dma_fd = -1; 

// ==========================================
// 极致零拷贝的 DMA 内存结构体与分配器
// ==========================================
struct DmaBuffer {
    void* virt_addr; 
    uint64_t size;   
    uint32_t handle; 
    int dma_fd;      
};

int alloc_drm_dma_buffer(int drm_fd, uint32_t width, uint32_t height, uint32_t bpp, DmaBuffer* buf) {
    struct drm_mode_create_dumb create = {0};
    create.width = width;
    create.height = height;
    create.bpp = bpp; 
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
    buf->virt_addr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
    if (buf->virt_addr == MAP_FAILED) return -1;
    if (drmPrimeHandleToFD(drm_fd, buf->handle, DRM_CLOEXEC, &buf->dma_fd) < 0) return -1;
    memset(buf->virt_addr, 0, buf->size); 
    return 0;
}

void free_drm_dma_buffer(int drm_fd, DmaBuffer* buf) {
    if (buf->dma_fd > 0) close(buf->dma_fd);
    if (buf->virt_addr && buf->virt_addr != MAP_FAILED) munmap(buf->virt_addr, buf->size);
    if (buf->handle > 0) {
        struct drm_mode_destroy_dumb destroy = {0};
        destroy.handle = buf->handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
}

// ==========================================
// 统一的 MAIN 函数
// ==========================================
int main(int argc, char **argv)
{
    if (argc != 2) { 
        printf("用法: %s <yolov5.rknn>\n", argv[0]); 
        return -1; 
    }

    // ==========================================
    // 0. 异步初始化 MQTT (跑在后台线程)
    // ==========================================
    mosquitto_lib_init();
    mqtt_client = mosquitto_new("rk3566_ai_terminal", true, NULL);
    if (mqtt_client) {
        mosquitto_connect_callback_set(mqtt_client, on_connect);
        mosquitto_message_callback_set(mqtt_client, on_message);
        mosquitto_username_pw_set(mqtt_client, MQTT_USER, MQTT_PWD);
        mosquitto_tls_set(mqtt_client, CA_CERT_PATH, NULL, NULL, NULL, NULL);
        
        printf("[0] 正在后台连接 MQTT Broker: %s...\n", MQTT_HOST);
        mosquitto_connect_async(mqtt_client, MQTT_HOST, MQTT_PORT, 60);
        // 【关键】：使用 start 开启后台线程，不阻塞当前代码继续执行！
        mosquitto_loop_start(mqtt_client); 
    }

    // ==========================================
    // 1. 初始化 YOLOv5
    // ==========================================
    init_post_process();
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
    memset(fb_ptr, 0, fb_size); 

    drmModeAddFB(drm_fd, SCREEN_WIDTH, SCREEN_HEIGHT, 24, 32, pitch, gem_handle, &fb_id);
    drmModeSetCrtc(drm_fd, crtc_id, fb_id, 0, 0, &conn->connector_id, 1, &conn->modes[0]);
    
    if (drmPrimeHandleToFD(drm_fd, gem_handle, DRM_CLOEXEC, &screen_dma_fd) == 0) {
        printf("[2] DRM 初始化成功，屏幕 DMA_FD: %d\n", screen_dma_fd);
    }

    // ==========================================
    // 3. 初始化 V4L2 摄像头
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
        struct v4l2_plane planes[1] = {0};
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;

        ioctl(cam_fd, VIDIOC_QUERYBUF, &buf);
        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buf.m.planes[0].m.mem_offset);
        
        struct v4l2_exportbuffer expbuf;
        memset(&expbuf, 0, sizeof(expbuf));
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        expbuf.index = i;
        if (ioctl(cam_fd, VIDIOC_EXPBUF, &expbuf) == 0) buffers[i].dma_fd = expbuf.fd;
        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(cam_fd, VIDIOC_STREAMON, &type);
    printf("[3] 摄像头流水线构建完毕！\n");

    // ==========================================
    // 4. 中转内存池分配
    // ==========================================
    DmaBuffer model_dma = {0};
    DmaBuffer canvas_dma = {0};
    alloc_drm_dma_buffer(drm_fd, MODEL_WIDTH, MODEL_HEIGHT, 24, &model_dma);
    alloc_drm_dma_buffer(drm_fd, CAM_WIDTH, CAM_HEIGHT, 24, &canvas_dma);
    
    image_buffer_t rknn_img = {0};
    rknn_img.width = MODEL_WIDTH; rknn_img.height = MODEL_HEIGHT;
    rknn_img.width_stride = MODEL_WIDTH; rknn_img.height_stride = MODEL_HEIGHT;
    rknn_img.format = IMAGE_FORMAT_RGB888;
    rknn_img.virt_addr = (unsigned char*)model_dma.virt_addr; 
    rknn_img.fd = model_dma.dma_fd;

    image_buffer_t canvas_img = {0};
    canvas_img.width = CAM_WIDTH; canvas_img.height = CAM_HEIGHT;
    canvas_img.width_stride = CAM_WIDTH; canvas_img.height_stride = CAM_HEIGHT;
    canvas_img.format = IMAGE_FORMAT_RGB888;
    canvas_img.virt_addr = (unsigned char*)canvas_dma.virt_addr; 
    canvas_img.fd = canvas_dma.dma_fd;

    printf("[4] DMA 中转内存分配成功！\n");

    // ==========================================
    // 5. 终极主循环
    // ==========================================
    printf("--- 开始 AI 视觉推理循环 ---\n");
    struct timeval start_time, end_time;
    struct timeval last_publish_time = {0}; // 记录上次发送MQTT的时间
    int frame_count = 0;
    
    // 初始化 TurboJPEG 压缩器实例 (循环外初始化，提高性能)
    tjhandle tj_compressor = tjInitCompress();

    gettimeofday(&start_time, NULL);

    // 假设死循环跑下去，或者设定一个大数字
    for (int frame = 0; frame < 50000; ++frame) {
        struct v4l2_plane planes[1] = {0};
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(cam_fd, VIDIOC_DQBUF, &buf) < 0) break;

        // RGA 处理 1
        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(buffers[buf.index].dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
        imresize(rga_cam_nv12, rga_model_rgb); 

        // AI 推理
        object_detect_result_list od_results;
        inference_yolov5_model(&rknn_app_ctx, &rknn_img, &od_results);

        // RGA 处理 2
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
        imcopy(rga_cam_nv12, rga_canvas_rgb);

        char text[256]; 
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result* det = &od_results.results[i];
            int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
            int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
            int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));
            
            draw_rectangle(&canvas_img, x1, y1, w, h, COLOR_BLUE, 3);
            sprintf(text, "%s %.1f%%", coco_cls_to_name(det->cls_id), det->prop * 100);
            int text_y = (y1 - 20 < 10) ? (y1 + 10) : (y1 - 20); 
            draw_text(&canvas_img, text, x1, text_y, COLOR_RED, 10);
        }

        // ========================================================
        // 【核心新增】：触发条件 -> 画面中有物体，且距上次发送超过 3 秒
        // ========================================================
        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        float time_since_last_pub = (current_time.tv_sec - last_publish_time.tv_sec) + 
                                    (current_time.tv_usec - last_publish_time.tv_usec) / 1000000.0f;

        if (od_results.count > 0 && time_since_last_pub > 3.0f && mqtt_client != NULL) {
            
            // 1. 将画好框的 RGB 内存直接压缩为 JPEG 内存
            unsigned char *jpeg_buf = NULL;
            unsigned long jpeg_size = 0;
            // 参数：源地址，宽，跨距(0默认)，高，源格式，输出指针，输出大小，采样，质量(0-100)，标志
            int tj_stat = tjCompress2(tj_compressor, canvas_img.virt_addr, CAM_WIDTH, 0, CAM_HEIGHT, 
                                      TJPF_RGB, &jpeg_buf, &jpeg_size, TJSAMP_422, 75, TJFLAG_FASTDCT);
            
            if (tj_stat == 0 && jpeg_size > 0) {
                // 2. 将 JPEG 内存数据用 OpenSSL 转换为 Base64
                // Base64 编码后的大小预估: 4 * ((原始大小 + 2) / 3) + 1 (留给'\0')
                size_t b64_max_len = 4 * ((jpeg_size + 2) / 3) + 1;
                char *b64_buf = (char *)malloc(b64_max_len);
                
                // EVP_EncodeBlock 不会自动插入烦人的换行符，非常适合生成 JSON 字段
                int b64_len = EVP_EncodeBlock((unsigned char *)b64_buf, jpeg_buf, jpeg_size);
                
                // 3. 拼装为 JSON 并发送 (动态分配大内存存 JSON)
                size_t json_size = b64_len + 512; // 预留 512 字节给 JSON 格式的其他部分
                char *json_buf = (char *)malloc(json_size);
                
                // 组装格式 (携带 MIME 头，方便部分前端直接解析 data:image/jpeg;base64,xxx)
                snprintf(json_buf, json_size, 
                         "{\"timestamp\": %ld, \"target_count\": %d, \"image\": \"data:image/jpeg;base64,%s\"}", 
                         current_time.tv_sec, od_results.count, b64_buf);

                // 发送！QoS=0，非阻塞发送
                mosquitto_publish(mqtt_client, NULL, MQTT_TOPIC, strlen(json_buf), json_buf, 0, false);
                printf(">> [MQTT] 已上报事件与抓拍图 (JPEG size: %lu bytes)!\n", jpeg_size);

                // 释放临时申请的内存
                free(json_buf);
                free(b64_buf);
                tjFree(jpeg_buf); // 注意：TurboJPEG 分配的内存要用 tjFree 释放

                // 更新冷却计时器
                last_publish_time = current_time;
            } else {
                printf("[Error] JPEG 压缩失败: %s\n", tjGetErrorStr());
            }
        }

        // RGA 处理 3: 刷屏
        int fb_format = RK_FORMAT_BGRA_8888;
        rga_buffer_t dst_fb = wrapbuffer_fd(screen_dma_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        ioctl(cam_fd, VIDIOC_QBUF, &buf);

        frame_count++;
        if (frame_count % 30 == 0) {
            gettimeofday(&end_time, NULL);
            float elapsed = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec) / 1000000.0f;
            printf("当前全链路平均 FPS: %.2f \n", 30.0f / elapsed);
            gettimeofday(&start_time, NULL); 
        }
    }

    // ==========================================
    // 6. 清理战场 (释放资源)
    // ==========================================
    printf("--- 程序结束，开始清理资源 ---\n");
    
    // 清理 MQTT
    if (mqtt_client) {
        mosquitto_disconnect(mqtt_client);
        mosquitto_loop_stop(mqtt_client, true); // 停止后台网络线程
        mosquitto_destroy(mqtt_client);
    }
    mosquitto_lib_cleanup();

    // 清理 TurboJPEG
    tjDestroy(tj_compressor);
    
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < REQ_BUF_CNT; ++i) {
        if (buffers[i].dma_fd > 0) close(buffers[i].dma_fd); 
        if (buffers[i].start != NULL && buffers[i].start != MAP_FAILED) munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
    struct v4l2_requestbuffers req_free = {0};
    req_free.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req_free.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_REQBUFS, &req_free);

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