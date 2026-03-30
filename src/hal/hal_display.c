#include "hal_display.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h> // 【新增】用于阻塞等待事件

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

// 【修改】：使用宏定义双缓冲数量
#define NUM_BUFFERS 2

static int drm_fd = -1;
static uint32_t g_crtc_id = 0; // 保存 CRTC ID 供 Page Flip 使用

// 【修改】：全部改成数组
static uint32_t fb_id[NUM_BUFFERS];
static uint32_t gem_handle[NUM_BUFFERS];
static void* fb_ptr[NUM_BUFFERS];
static uint64_t fb_size = 0;
static int screen_dma_fd[NUM_BUFFERS]; 

static drmModeCrtc *saved_crtc = NULL;
static drmModeConnector *conn = NULL;
static drmModeRes *res = NULL;

// 【新增】：双缓冲状态机指针
static int front_buf_idx = 0;
static int back_buf_idx = 1;
static int page_flip_pending = 0;

// 【新增】：DRM 翻页完成的硬件中断回调函数
static void page_flip_handler(int fd, unsigned int frame, unsigned int sec, unsigned int usec, void *data) {
    // 硬件说：翻页完成了！清除挂起标志。
    page_flip_pending = 0;
}

int hal_display_init(void) {
    drm_fd = open("/dev/dri/card0", O_RDWR);
    if (drm_fd < 0) {
        printf("[HAL Display] Error: 打开 /dev/dri/card0 失败\n");
        return -1;
    }

    res = drmModeGetResources(drm_fd);
    conn = drmModeGetConnector(drm_fd, res->connectors[0]);
    g_crtc_id = res->crtcs[0];  
    saved_crtc = drmModeGetCrtc(drm_fd, g_crtc_id);

    // 【修改】：循环 2 次，申请 2 块 Dumb Buffer
    for (int i = 0; i < NUM_BUFFERS; i++) {
        struct drm_mode_create_dumb create = {0};
        create.width = SCREEN_WIDTH; 
        create.height = SCREEN_HEIGHT; 
        create.bpp = 32;
        if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) return -1;
        
        gem_handle[i] = create.handle; 
        fb_size = create.size; // 假设两次申请大小一致
        
        struct drm_mode_map_dumb map = {0}; 
        map.handle = gem_handle[i];
        if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) return -1;
        
        fb_ptr[i] = mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
        memset(fb_ptr[i], 0, fb_size); 

        // 注册到 DRM 获得 FB ID
        drmModeAddFB(drm_fd, SCREEN_WIDTH, SCREEN_HEIGHT, 24, 32, create.pitch, gem_handle[i], &fb_id[i]);
        // 导出 DMA FD
        drmPrimeHandleToFD(drm_fd, gem_handle[i], DRM_CLOEXEC, &screen_dma_fd[i]);
    }

    // 初始化先点亮第一块屏幕 (Front Buffer)
    drmModeSetCrtc(drm_fd, g_crtc_id, fb_id[front_buf_idx], 0, 0, &conn->connector_id, 1, &conn->modes[0]);

    printf("[HAL Display] DRM 双缓冲 VSync 框架启动完毕！\n");
    return 0;
}

int hal_display_get_back_buffer_fd(void) {
    // 永远把后台空闲的那块画布交给 RGA
    return screen_dma_fd[back_buf_idx];
}

void hal_display_commit_and_wait(void) {
    // 1. 发起翻页请求：把刚才画好的 Back Buffer 提交给屏幕，要求在下一次 VSync 时翻页
    page_flip_pending = 1;
    drmModePageFlip(drm_fd, g_crtc_id, fb_id[back_buf_idx], DRM_MODE_PAGE_FLIP_EVENT, NULL);

    // 2. 配置事件监听器
    drmEventContext evctx = {0};
    evctx.version = DRM_EVENT_CONTEXT_VERSION;
    evctx.page_flip_handler = page_flip_handler;

    fd_set fds;
    
    // 3. 阻塞等待：Linux 线程会在这里休眠，直到硬件屏幕扫完这一帧产生中断唤醒它
    while (page_flip_pending) {
        FD_ZERO(&fds);
        FD_SET(drm_fd, &fds);
        
        int ret = select(drm_fd + 1, &fds, NULL, NULL, NULL);
        if (ret > 0) {
            if (FD_ISSET(drm_fd, &fds)) {
                // 读取内核事件，这会触发上面的 page_flip_handler
                drmHandleEvent(drm_fd, &evctx); 
            }
        } else if (ret == 0) {
            break; // Timeout防死锁 (实际没配超时不应该走到这里)
        }
    }

    // 4. 翻页成功！角色互换。原来的 Back 变成了现在的 Front，原来的 Front 拿来当新的 Back。
    front_buf_idx = back_buf_idx;
    back_buf_idx = 1 - back_buf_idx; 
}

// ... 下方的 alloc_buffer 和 free_buffer 保持原样 ...
int hal_display_alloc_buffer(uint32_t width, uint32_t height, uint32_t bpp, hal_drm_buf_t* buf) {
    struct drm_mode_create_dumb create = {0};
    create.width = width; 
    create.height = height; 
    create.bpp = bpp; 
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

void hal_display_free_buffer(hal_drm_buf_t* buf) {
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