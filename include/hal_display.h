#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void* virt_addr;
    uint64_t size;
    uint32_t handle;
    int dma_fd;
} hal_drm_buf_t;

int hal_display_init(void);

// 获取当前后台画布（Back Buffer）的 DMA FD，供 RGA 画图用
int hal_display_get_back_buffer_fd(void);

// 提交后台画布到屏幕，并阻塞等待 VSync 垂直同步信号到来
void hal_display_commit_and_wait(void);

int hal_display_alloc_buffer(uint32_t width, uint32_t height, uint32_t bpp, hal_drm_buf_t* buf);
void hal_display_free_buffer(hal_drm_buf_t* buf);
void hal_display_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_DISPLAY_H