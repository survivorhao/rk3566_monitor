#include "hal_display.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h> 

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

// frame buffer数量
#define NUM_BUFFERS 2

static int drm_fd = -1;     
static uint32_t g_crtc_id = 0; // 保存 CRTC ID 供 Page Flip 使用

// 全部改成数组
static uint32_t fb_id[NUM_BUFFERS];
static uint32_t gem_handle[NUM_BUFFERS];
static void* fb_ptr[NUM_BUFFERS];
static uint64_t fb_size = 0;
static int screen_dma_fd[NUM_BUFFERS]; 

static drmModeCrtc *saved_crtc = NULL;
static drmModeConnector *conn = NULL;
static drmModeRes *res = NULL;

// 双缓冲状态机指针
static int front_buf_idx = 0;
static int back_buf_idx = 1;
static int page_flip_pending = 0;

// DRM 翻页完成的硬件中断回调函数
static void page_flip_handler(int fd, unsigned int frame, unsigned int sec, unsigned int usec, void *data) {
    // 硬件说：翻页完成了！清除挂起标志。
    page_flip_pending = 0;

    //printf("frame_id=%d, sec %u,usec %u \n",frame,sec, usec);
    
}




/**
 * @brief  初始化drm subsystem,申请两个dumb buffer
 * 
 * @return int 
 */
int hal_display_init(void) {

    int i=0;
    //直接打开drm primary结点
    drm_fd = open("/dev/dri/card0", O_RDWR);
    if (drm_fd < 0) {
        printf("[HAL Display] Error: open /dev/dri/card0 fail,please whether support drm\n");
        return -1;
    }

    //获得所有的fb,crtc,connector,enconder id
    res = drmModeGetResources(drm_fd);
    
    //多个connector的情况下，需要查找当前建立连接的connector
    for(;i<res->count_connectors;i++)
    {
        conn = drmModeGetConnector(drm_fd, res->connectors[i]);

        if(conn->connection==DRM_MODE_CONNECTED)
        {
            break;
        }
    }


    //rk3566中video output processor的一个video port对应一个CRTC
    //我们这边在设备树中仅仅enable了一个video port
    //因此这边直接使用，理论上需要从connector找encoder,进而从encoder找CRTC
    if(res->count_crtcs==1)
    {
        g_crtc_id = res->crtcs[0];  
        saved_crtc = drmModeGetCrtc(drm_fd, g_crtc_id);
    }



    // 采用double buffer
    for (int i = 0; i < NUM_BUFFERS; i++) {
        struct drm_mode_create_dumb create = {0};
        create.width = SCREEN_WIDTH; 
        create.height = SCREEN_HEIGHT; 
        create.bpp = 32;     //bits per pixel,每个像素需要多少bits位表示

        //申请dumb buffer
        if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) return -1;
        
        //保存驱动返回的一些数据
        gem_handle[i] = create.handle; //这个dumb buffer在驱动中对于的Id 
        fb_size = create.size;         //这个dumb buffer实际的size
        
        
        struct drm_mode_map_dumb map = {0}; 
        map.handle = gem_handle[i];
        //为后续mmap，将这个dumb buffer映射到用户空间供cpu读写做准备
        //这个函数会设置一个特殊的offset
        if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) return -1;
        
        //映射到用户空间，传递这个特殊的offset，驱动找到这个特殊的Offset所对应的dumb buffer
        fb_ptr[i] = mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
        memset(fb_ptr[i], 0, fb_size); 

        // 注册到 DRM 获得frame buffer id,这是一个特殊的id
        //驱动通过这个特殊的fb id可找到对应的frame buffer
        drmModeAddFB(drm_fd, SCREEN_WIDTH, SCREEN_HEIGHT, 24, 32, create.pitch, gem_handle[i], &fb_id[i]);
        
        
        //导出为一个dma fd file descriptor
        drmPrimeHandleToFD(drm_fd, gem_handle[i], DRM_CLOEXEC, &screen_dma_fd[i]);
    }

    //采用double buffer,先使用第一个fb
    drmModeSetCrtc(drm_fd, g_crtc_id, fb_id[front_buf_idx], 0, 0, &conn->connector_id, 1, &conn->modes[0]);

    printf("[HAL Display] DRM double buffer VSync framework start success !\n");
    return 0;
}



int hal_display_get_back_buffer_fd(void) {
    // 永远把后台空闲的那块画布交给 RGA
    return screen_dma_fd[back_buf_idx];
}

#include <time.h>
#include <stdio.h>
#include <sys/select.h>

// 获取单调递增的系统时间（精确到毫秒）
static inline double hal_get_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

void hal_display_commit_and_wait(void) {
    page_flip_pending = 1;
    
    // ==========================================================
    // 【探针起点】：记录准备呼叫 DRM 驱动的时间点
    // ==========================================================
    double start_time = hal_get_time_ms();

    //每次在vsync事件到来时，才进行buffer的切换
    //请求驱动在vblank事件发生之后向用户空间发送DRM_MODE_PAGE_FLIP_EVENT事件
    drmModePageFlip(drm_fd, g_crtc_id, fb_id[back_buf_idx], DRM_MODE_PAGE_FLIP_EVENT, NULL);

    //配置事件监听器
    drmEventContext evctx = {0};
    evctx.version = DRM_EVENT_CONTEXT_VERSION;
    evctx.page_flip_handler = page_flip_handler;

    fd_set fds;
    
    // 调用select进行阻塞等待
    while (page_flip_pending) {
        FD_ZERO(&fds);
        FD_SET(drm_fd, &fds);
        
        int ret = select(drm_fd + 1, &fds, NULL, NULL, NULL);
        if (ret > 0) {
            if (FD_ISSET(drm_fd, &fds)) {
                //处理DRM_MODE_PAGE_FLIP_EVENT事件 (成功后会把 page_flip_pending 置 0)
                drmHandleEvent(drm_fd, &evctx); 
            }
        } else if (ret == 0) {
            break; // Timeout防死锁 
        }
    }

    // ==========================================================
    // 【探针终点】：成功等到 VSync 信号，唤醒出站
    // ==========================================================
    double end_time = hal_get_time_ms();
    double wait_time = end_time - start_time;

    // 局部静态变量，用来统计 60 帧的平均等待时间
    static int flip_count = 0;
    static double total_wait_time = 0;
    flip_count++;
    total_wait_time += wait_time;

    if (flip_count == 60) {
        printf("[Profiler] ⏳ DRM VSync Wait Avg: %.2f ms\n", total_wait_time / 60.0);
        flip_count = 0;
        total_wait_time = 0;
    }

    // 4. 翻页成功！角色互换。原来的 Back 变成了现在的 Front，原来的 Front 拿来当新的 Back。
    front_buf_idx = back_buf_idx;
    back_buf_idx = 1 - back_buf_idx; 
}
/**
 * @brief   使用drm subsystem提供的方式，在用户空间实现高效的分配graphical buffer 
 * 
 * @param width 
 * @param height 
 * @param bpp  bits per pixel
 * @param buf 
 * @return int 0 on success,
 */
int hal_display_alloc_buffer(uint32_t width, uint32_t height, uint32_t bpp, hal_drm_buf_t* buf) 
{
    struct drm_mode_create_dumb create = {0};
    create.width = width; 
    create.height = height; 
    create.bpp = bpp; //bits per pixel,每个像素需要多少bits位表示
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) return -1;
    
    buf->size = create.size; 
    buf->handle = create.handle;
    
    struct drm_mode_map_dumb map = {0}; 
    map.handle = buf->handle;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) return -1;
    
    buf->virt_addr = mmap(NULL, buf->size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
    if (buf->virt_addr == MAP_FAILED) return -1;
    
    if (drmPrimeHandleToFD(drm_fd, buf->handle, DRM_CLOEXEC, &buf->dma_fd) < 0) return -1;
    memset(buf->virt_addr, 0, buf->size); 
    return 0;
}
/**
 * @brief free graphical buffer 
 * 
 * @param buf 
 */
void hal_display_free_buffer(hal_drm_buf_t* buf) 
{
    if (buf->dma_fd > 0) close(buf->dma_fd);
    if (buf->virt_addr && buf->virt_addr != MAP_FAILED) munmap(buf->virt_addr, buf->size);
    if (buf->handle > 0) {
        struct drm_mode_destroy_dumb destroy = {0}; 
        destroy.handle = buf->handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
}

void hal_display_deinit(void) {
    if (saved_crtc) {
        drmModeSetCrtc(drm_fd, saved_crtc->crtc_id, saved_crtc->buffer_id,
                       saved_crtc->x, saved_crtc->y, &conn->connector_id, 1, &saved_crtc->mode);
        drmModeFreeCrtc(saved_crtc);
    }
    
    // 释放双份资源
    for(int i = 0; i < NUM_BUFFERS; i++) {
        if (fb_id[i]) drmModeRmFB(drm_fd, fb_id[i]);
        if (gem_handle[i]) {
            struct drm_mode_destroy_dumb destroy = {0}; 
            destroy.handle = gem_handle[i];
            drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        }
        if (fb_ptr[i] && fb_ptr[i] != MAP_FAILED) munmap(fb_ptr[i], fb_size);
        if (screen_dma_fd[i] > 0) close(screen_dma_fd[i]);
    }

    if (conn) drmModeFreeConnector(conn);
    if (res) drmModeFreeResources(res);
    if (drm_fd >= 0) close(drm_fd);
}