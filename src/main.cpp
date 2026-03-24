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

// === 新增：多线程与事件监听 ===
#include <pthread.h>
#include <sys/epoll.h>
#include <errno.h>

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
// 【配置】：传感器节点与 MQTT 参数
// ==========================================
#define SR501_DEV "/dev/sr501_dev0"
#define SGP30_DEV "/dev/sgp30_0"
#define DHT11_TEMP "/sys/bus/iio/devices/iio:device0/in_temp_input"
#define DHT11_HUMI "/sys/bus/iio/devices/iio:device0/in_humidityrelative_input"
#define MAX_EPOLL_EVENTS 5

#define MQTT_HOST       "z9c1fa31.ala.cn-hangzhou.emqxsl.cn"
#define MQTT_PORT       8883
#define MQTT_USER       "rk3566dev"
#define MQTT_PWD        "2cpvhZLTX8UfX4R"
#define CA_CERT_PATH    "./emqxsl-ca.crt"
#define MQTT_TOPIC      "rk3566/ai_events"

struct mosquitto *mqtt_client = NULL;

bool g_mqtt_connected = false; // 【新增】MQTT 实时连接状态标志
// ==========================================
// 全局状态机与传感器数据共享区 (含线程锁)
// ==========================================
struct SensorState {
    int co2;
    int tvoc;
    float temp;
    float humi;
    bool is_ai_active;
    uint64_t ai_timeout_ms;
};
SensorState g_sensor_state = {0, 0, 0.0f, 0.0f, false, 0};
pthread_mutex_t g_state_mutex = PTHREAD_MUTEX_INITIALIZER;

uint64_t get_current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

int read_sysfs_int(const char *path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    char buf[32] = {0};
    read(fd, buf, sizeof(buf) - 1);
    close(fd);
    return atoi(buf);
}

// ==========================================
// 后台传感器监听线程
// ==========================================
void* sensor_thread_func(void* arg) {
    int epfd, sr501_fd, sgp30_fd;
    struct epoll_event ev, events[MAX_EPOLL_EVENTS];

    sr501_fd = open(SR501_DEV, O_RDONLY | O_NONBLOCK);
    sgp30_fd = open(SGP30_DEV, O_RDONLY | O_NONBLOCK);
    
    if (sr501_fd < 0 || sgp30_fd < 0) {
        printf("[Sensor Thread] 警告：无法打开传感器节点，请确保驱动已加载。\n");
        return NULL;
    }

    epfd = epoll_create1(0);
    ev.events = EPOLLIN;
    ev.data.fd = sr501_fd;
    epoll_ctl(epfd, EPOLL_CTL_ADD, sr501_fd, &ev);

    ev.events = EPOLLIN;
    ev.data.fd = sgp30_fd;
    epoll_ctl(epfd, EPOLL_CTL_ADD, sgp30_fd, &ev);

    printf("[Sensor Thread] 后台监听引擎已启动...\n");

    while (1) {
        int nfds = epoll_wait(epfd, events, MAX_EPOLL_EVENTS, -1);
        if (nfds < 0) {
            if (errno == EINTR) continue;
            break;
        }

        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd == sr501_fd) {
                char buf[4] = {0};
                if (read(sr501_fd, buf, sizeof(buf)) > 0) {
                    pthread_mutex_lock(&g_state_mutex);
                    g_sensor_state.is_ai_active = true;
                    g_sensor_state.ai_timeout_ms = get_current_time_ms() + 10000; // 设置10秒看门狗
                    pthread_mutex_unlock(&g_state_mutex);
                    printf("\n🚨 [SR501] 检测到移动，唤醒 AI 引擎 10 秒！\n");
                }
            }
            else if (events[i].data.fd == sgp30_fd) {
                int data[2] = {0};
                if (read(sgp30_fd, data, sizeof(data)) == sizeof(data)) {
                    int t_raw = read_sysfs_int(DHT11_TEMP);
                    int h_raw = read_sysfs_int(DHT11_HUMI);
                    
                    printf("tem %d,hum %d \n",t_raw,h_raw);
                    
                    pthread_mutex_lock(&g_state_mutex);
                    g_sensor_state.co2 = data[0];
                    g_sensor_state.tvoc = data[1];
                    g_sensor_state.temp = t_raw / 1000.0f;
                    g_sensor_state.humi = h_raw / 1000.0f;
                    pthread_mutex_unlock(&g_state_mutex);
                }
            }
        }
    }
    return NULL;
}


// MQTT 连接成功的回调函数
void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        printf("[MQTT] 成功通过 TLS 连接到 EMQX Broker!\n");
        g_mqtt_connected = true; // 连上了，标志置位
    } else {
        printf("[MQTT] 连接失败，错误码: %d\n", rc);
        g_mqtt_connected = false;
    }
}

// 【新增】MQTT 意外断开的回调函数 (支持底层自动重连)
void on_disconnect(struct mosquitto *mosq, void *obj, int rc) {
    printf("[MQTT] 警告：与服务器的连接已断开 (原因码: %d)！\n", rc);
    g_mqtt_connected = false; // 掉线了，标志复位
}
// ==========================================
// V4L2 与 DRM DMA 结构体 (保持不变)
// ==========================================
struct CamBuffer { void* start; size_t length; int dma_fd; };
CamBuffer* buffers = NULL;
int cam_fd = -1;

int drm_fd = -1;
uint32_t fb_id = 0, gem_handle = 0, pitch = 0;
void* fb_ptr = NULL; uint64_t fb_size = 0;
drmModeCrtc *saved_crtc = NULL;
int screen_dma_fd = -1; 

struct DmaBuffer { void* virt_addr; uint64_t size; uint32_t handle; int dma_fd; };

int alloc_drm_dma_buffer(int drm_fd, uint32_t width, uint32_t height, uint32_t bpp, DmaBuffer* buf) {
    struct drm_mode_create_dumb create = {0};
    create.width = width; create.height = height; create.bpp = bpp; 
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) return -1;
    buf->size = create.size; buf->handle = create.handle;
    struct drm_mode_map_dumb map = {0}; map.handle = buf->handle;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) return -1;
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
        struct drm_mode_destroy_dumb destroy = {0}; destroy.handle = buf->handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
}

// ==========================================
// MAIN 函数
// ==========================================
int main(int argc, char **argv)
{
    if (argc != 2) { printf("用法: %s <yolov5.rknn>\n", argv[0]); return -1; }

    // 0. 启动 MQTT
    mosquitto_lib_init();
    mqtt_client = mosquitto_new("rk3566_ai_terminal", true, NULL);
    if (mqtt_client) {
        mosquitto_connect_callback_set(mqtt_client, on_connect);
        mosquitto_disconnect_callback_set(mqtt_client, on_disconnect); // 【新增】注册断线回调
        mosquitto_username_pw_set(mqtt_client, MQTT_USER, MQTT_PWD);
        mosquitto_tls_set(mqtt_client, CA_CERT_PATH, NULL, NULL, NULL, NULL);
        mosquitto_connect_async(mqtt_client, MQTT_HOST, MQTT_PORT, 60);
        mosquitto_loop_start(mqtt_client); 
    }


    // 0.5 启动传感器后台线程
    pthread_t sensor_tid;
    pthread_create(&sensor_tid, NULL, sensor_thread_func, NULL);
    pthread_detach(sensor_tid);

    // 1. 初始化 YOLOv5
    init_post_process();
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    if (init_yolov5_model(argv[1], &rknn_app_ctx) != 0) return -1;

    // 2. DRM 初始化
    drm_fd = open("/dev/dri/card0", O_RDWR);
    drmModeRes *res = drmModeGetResources(drm_fd);
    drmModeConnector *conn = drmModeGetConnector(drm_fd, res->connectors[0]);
    uint32_t crtc_id = res->crtcs[0];  
    saved_crtc = drmModeGetCrtc(drm_fd, crtc_id);

    struct drm_mode_create_dumb create = {0};
    create.width = SCREEN_WIDTH; create.height = SCREEN_HEIGHT; create.bpp = 32;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create);
    gem_handle = create.handle; pitch = create.pitch; fb_size = create.size;
    struct drm_mode_map_dumb map = {0}; map.handle = gem_handle;
    drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map);
    fb_ptr = mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
    memset(fb_ptr, 0, fb_size); 

    drmModeAddFB(drm_fd, SCREEN_WIDTH, SCREEN_HEIGHT, 24, 32, pitch, gem_handle, &fb_id);
    drmModeSetCrtc(drm_fd, crtc_id, fb_id, 0, 0, &conn->connector_id, 1, &conn->modes[0]);
    drmPrimeHandleToFD(drm_fd, gem_handle, DRM_CLOEXEC, &screen_dma_fd);

    // 3. V4L2 初始化
    cam_fd = open(CAM_DEV, O_RDWR);
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = CAM_WIDTH; fmt.fmt.pix_mp.height = CAM_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12; 
    ioctl(cam_fd, VIDIOC_S_FMT, &fmt);

    struct v4l2_requestbuffers req = {0};
    req.count = REQ_BUF_CNT; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; req.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_REQBUFS, &req);

    buffers = (CamBuffer*)calloc(req.count, sizeof(CamBuffer));
    for (int i = 0; i < req.count; ++i) {
        struct v4l2_plane planes[1] = {0};
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; buf.memory = V4L2_MEMORY_MMAP; buf.index = i; buf.m.planes = planes; buf.length = 1;
        ioctl(cam_fd, VIDIOC_QUERYBUF, &buf);
        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buf.m.planes[0].m.mem_offset);
        struct v4l2_exportbuffer expbuf = {0}; expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; expbuf.index = i;
        ioctl(cam_fd, VIDIOC_EXPBUF, &expbuf); buffers[i].dma_fd = expbuf.fd;
        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(cam_fd, VIDIOC_STREAMON, &type);

    // 4. DMA 内存池
    DmaBuffer model_dma = {0}; DmaBuffer canvas_dma = {0};
    alloc_drm_dma_buffer(drm_fd, MODEL_WIDTH, MODEL_HEIGHT, 24, &model_dma);
    alloc_drm_dma_buffer(drm_fd, CAM_WIDTH, CAM_HEIGHT, 24, &canvas_dma);
    
    image_buffer_t rknn_img = {0};
    rknn_img.width = MODEL_WIDTH; rknn_img.height = MODEL_HEIGHT; rknn_img.width_stride = MODEL_WIDTH; rknn_img.height_stride = MODEL_HEIGHT;
    rknn_img.format = IMAGE_FORMAT_RGB888; rknn_img.virt_addr = (unsigned char*)model_dma.virt_addr; rknn_img.fd = model_dma.dma_fd;

    image_buffer_t canvas_img = {0};
    canvas_img.width = CAM_WIDTH; canvas_img.height = CAM_HEIGHT; canvas_img.width_stride = CAM_WIDTH; canvas_img.height_stride = CAM_HEIGHT;
    canvas_img.format = IMAGE_FORMAT_RGB888; canvas_img.virt_addr = (unsigned char*)canvas_dma.virt_addr; canvas_img.fd = canvas_dma.dma_fd;

    // 5. 终极主循环
    printf("--- 开始 AI 视觉推理循环 (基于状态机) ---\n");
    struct timeval last_publish_time = {0}; 
    tjhandle tj_compressor = tjInitCompress();

    for (int frame = 0; frame < 50000; ++frame) {
        struct v4l2_plane planes[1] = {0};
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; buf.memory = V4L2_MEMORY_MMAP; buf.m.planes = planes; buf.length = 1;

        if (ioctl(cam_fd, VIDIOC_DQBUF, &buf) < 0) break;

        uint64_t now = get_current_time_ms();
        bool run_ai = false;
        
        // --- 核心状态机判决 ---
        pthread_mutex_lock(&g_state_mutex);
        if (g_sensor_state.is_ai_active) {
            if (now > g_sensor_state.ai_timeout_ms) {
                g_sensor_state.is_ai_active = false; // 超时休眠
                printf("[State] AI 激活已满 10 秒，未见人影，进入低功耗休眠。\n");
            } else {
                run_ai = true;
            }
        }
        // 快照一份当前传感器数据，用于 OSD 显示和云端上传
        float cur_temp = g_sensor_state.temp;
        float cur_humi = g_sensor_state.humi;
        int cur_co2 = g_sensor_state.co2;
        int cur_tvoc = g_sensor_state.tvoc;
        pthread_mutex_unlock(&g_state_mutex);

        // RGA 1: V4L2 NV12 转 RGB (仅用于画布复制)
        rga_buffer_t rga_cam_nv12 = wrapbuffer_fd(buffers[buf.index].dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_YCbCr_420_SP);
        rga_buffer_t rga_canvas_rgb = wrapbuffer_fd(canvas_dma.dma_fd, CAM_WIDTH, CAM_HEIGHT, RK_FORMAT_RGB_888);
        imcopy(rga_cam_nv12, rga_canvas_rgb); // 拷贝到画布，保证屏幕一直有预览图像

        object_detect_result_list od_results;
        od_results.count = 0; // 默认没检测到

        // --- 动态 AI 推理分支 ---
        if (run_ai) {
            rga_buffer_t rga_model_rgb = wrapbuffer_fd(model_dma.dma_fd, MODEL_WIDTH, MODEL_HEIGHT, RK_FORMAT_RGB_888);
            imresize(rga_cam_nv12, rga_model_rgb); 
            inference_yolov5_model(&rknn_app_ctx, &rknn_img, &od_results);

            if (od_results.count > 0) {
                // 只要检测到人，就刷新 10 秒看门狗！(续杯)
                pthread_mutex_lock(&g_state_mutex);
                g_sensor_state.ai_timeout_ms = now + 10000;
                pthread_mutex_unlock(&g_state_mutex);

                // 画检测框
                char text[256]; 
                for (int i = 0; i < od_results.count; i++) {
                    object_detect_result* det = &od_results.results[i];
                    int x1 = (int)(det->box.left * (CAM_WIDTH / (float)MODEL_WIDTH));
                    int y1 = (int)(det->box.top * (CAM_HEIGHT / (float)MODEL_HEIGHT));
                    int w = (int)((det->box.right - det->box.left) * (CAM_WIDTH / (float)MODEL_WIDTH));
                    int h = (int)((det->box.bottom - det->box.top) * (CAM_HEIGHT / (float)MODEL_HEIGHT));
                    
                    draw_rectangle(&canvas_img, x1, y1, w, h, 0xFF0000FF, 3); // Blue box
                    sprintf(text, "%s %.1f%%", coco_cls_to_name(det->cls_id), det->prop * 100);
                    int text_y = (y1 - 20 < 10) ? (y1 + 10) : (y1 - 20); 
                    draw_text(&canvas_img, text, x1, text_y, 0xFFFF0000, 10); // Red text
                }

                // 抓拍与上云 (冷却期 3 秒)
                struct timeval current_time;
                gettimeofday(&current_time, NULL);
                float time_since_last_pub = (current_time.tv_sec - last_publish_time.tv_sec) + 
                                            (current_time.tv_usec - last_publish_time.tv_usec) / 1000000.0f;

// 【修改】：必须加上 g_mqtt_connected == true 才能触发上报
                if (time_since_last_pub > 3.0f && mqtt_client != NULL && g_mqtt_connected) {
                    unsigned char *jpeg_buf = NULL;
                    unsigned long jpeg_size = 0;
                    if (tjCompress2(tj_compressor, canvas_img.virt_addr, CAM_WIDTH, 0, CAM_HEIGHT, 
                                    TJPF_RGB, &jpeg_buf, &jpeg_size, TJSAMP_422, 75, TJFLAG_FASTDCT) == 0) {
                        
                        size_t b64_max_len = 4 * ((jpeg_size + 2) / 3) + 1;
                        char *b64_buf = (char *)malloc(b64_max_len);
                        int b64_len = EVP_EncodeBlock((unsigned char *)b64_buf, jpeg_buf, jpeg_size);
                        size_t json_size = b64_len + 512; 
                        char *json_buf = (char *)malloc(json_size);
                        
                        snprintf(json_buf, json_size, 
                                 "{\"timestamp\": %ld, \"target_count\": %d, \"temp\": %.1f, \"humi\": %.1f, \"co2\": %d, \"tvoc\": %d, \"image\": \"data:image/jpeg;base64,%s\"}", 
                                 current_time.tv_sec, od_results.count, cur_temp, cur_humi, cur_co2, cur_tvoc, b64_buf);

                        // 【修改】：严格检查发送返回值
                        int pub_ret = mosquitto_publish(mqtt_client, NULL, MQTT_TOPIC, strlen(json_buf), json_buf, 0, false);
                        if (pub_ret == MOSQ_ERR_SUCCESS) {
                            printf(">> [MQTT] 已成功上报事件、传感器数据与抓拍图!\n");
                            last_publish_time = current_time; // 只有成功发送，才更新冷却计时器
                        } else {
                            printf(">> [MQTT Error] 发送失败！错误码：%d\n", pub_ret);
                        }

                        free(json_buf); free(b64_buf); tjFree(jpeg_buf);
                    }
                } 
                else if (time_since_last_pub > 3.0f && !g_mqtt_connected) {
                    // 【新增】：离线状态下的行为提示
                    printf(">> [Offline] 网络断开，检测到人体但无法上报，等待重连...\n");
                    last_publish_time = current_time; // 避免疯狂刷屏打印
                }
            }
        }

        // 画布左上角 OSD 状态显示 (无论是否唤醒都显示)
        char osd_text[256];
        sprintf(osd_text, "AI: %s | T:%.1fC H:%.1f%% CO2:%d", run_ai ? "ACTIVE" : "SLEEP", cur_temp, cur_humi, cur_co2);
        draw_text(&canvas_img, osd_text, 10, 30, run_ai ? 0xFFFF0000 : 0xFF00FF00, 2); // 激活显示红字，休眠显示绿字

        // RGA 3: 刷屏
        int fb_format = RK_FORMAT_BGRA_8888;
        rga_buffer_t dst_fb = wrapbuffer_fd(screen_dma_fd, SCREEN_WIDTH, SCREEN_HEIGHT, fb_format);
        imrotate(rga_canvas_rgb, dst_fb, IM_HAL_TRANSFORM_ROT_90);

        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }

    // 6. 清理资源 (略，保持原有退出逻辑)
    return 0;
}