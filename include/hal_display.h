#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 我们自己造的 DMA 内存池对象
typedef struct {
    void* virt_addr;
    uint64_t size;
    uint32_t handle;
    int dma_fd;
} hal_drm_buf_t;

// 1. 初始化 DRM 并在屏幕上生成一块画布
int hal_display_init(void);

// 2. 获取屏幕 DMA 句柄，供底层 RGA 刷屏用
int hal_display_get_screen_fd(void);

// 3. 在底层申请零拷贝 DMA 内存 (给模型和画布使用)
int hal_display_alloc_buffer(uint32_t width, uint32_t height, uint32_t bpp, hal_drm_buf_t* buf);

// 4. 释放 DMA 内存
void hal_display_free_buffer(hal_drm_buf_t* buf);

// 5. 销毁屏幕
void hal_display_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_DISPLAY_H