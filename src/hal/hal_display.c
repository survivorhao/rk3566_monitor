#include "hal_display.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

// DRM KMS API
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

static int drm_fd = -1;
static uint32_t fb_id = 0, gem_handle = 0, pitch = 0;
static void* fb_ptr = NULL; 
static uint64_t fb_size = 0;
static drmModeCrtc *saved_crtc = NULL;
static drmModeConnector *conn = NULL;
static drmModeRes *res = NULL;
static int screen_dma_fd = -1; 

int hal_display_init(void) {
    drm_fd = open("/dev/dri/card0", O_RDWR);
    if (drm_fd < 0) {
        printf("[HAL Display] Error: 打开 /dev/dri/card0 失败\n");
        return -1;
    }

    res = drmModeGetResources(drm_fd);
    conn = drmModeGetConnector(drm_fd, res->connectors[0]);
    uint32_t crtc_id = res->crtcs[0];  
    saved_crtc = drmModeGetCrtc(drm_fd, crtc_id);

    struct drm_mode_create_dumb create = {0};
    create.width = SCREEN_WIDTH; 
    create.height = SCREEN_HEIGHT; 
    create.bpp = 32;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) return -1;
    
    gem_handle = create.handle; 
    pitch = create.pitch; 
    fb_size = create.size;
    
    struct drm_mode_map_dumb map = {0}; 
    map.handle = gem_handle;
    if (drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) return -1;
    
    fb_ptr = mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd, map.offset);
    memset(fb_ptr, 0, fb_size); 

    drmModeAddFB(drm_fd, SCREEN_WIDTH, SCREEN_HEIGHT, 24, 32, pitch, gem_handle, &fb_id);
    drmModeSetCrtc(drm_fd, crtc_id, fb_id, 0, 0, &conn->connector_id, 1, &conn->modes[0]);
    drmPrimeHandleToFD(drm_fd, gem_handle, DRM_CLOEXEC, &screen_dma_fd);

    printf("[HAL Display] DRM 现代显示框架启动完毕！\n");
    return 0;
}

int hal_display_get_screen_fd(void) {
    return screen_dma_fd;
}

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
    if (fb_id) drmModeRmFB(drm_fd, fb_id);
    
    if (gem_handle) {
        struct drm_mode_destroy_dumb destroy = {0}; 
        destroy.handle = gem_handle;
        drmIoctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
    
    if (fb_ptr && fb_ptr != MAP_FAILED) munmap(fb_ptr, fb_size);
    if (screen_dma_fd > 0) close(screen_dma_fd);
    if (conn) drmModeFreeConnector(conn);
    if (res) drmModeFreeResources(res);
    if (drm_fd >= 0) close(drm_fd);
}