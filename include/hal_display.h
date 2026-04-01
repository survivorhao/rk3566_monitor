#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void* virt_addr;   //mmap之后，用户空间可以直接读写的虚拟地址
    uint64_t size;     //buffer size
    uint32_t handle;   //handle id, just a bumber,内核驱动会通过这个handle/id找到对应的数据,用户空间没有特殊含义
    int dma_fd;        //导出为一个dma file descriptor，用于实现硬件IP之间的zero copy
} hal_drm_buf_t;



/**
 * @brief  初始化drm subsystem,申请两个dumb buffer
 * 
 * @return int 
 */
int hal_display_init(void);

// 获取当前后台画布（Back Buffer）的 DMA FD，供 RGA 画图用
int hal_display_get_back_buffer_fd(void);

// 提交后台画布到屏幕，并阻塞等待 VSync 垂直同步信号到来
void hal_display_commit_and_wait(void);


/**
 * @brief   使用drm subsystem提供的方式，在用户空间实现高效的分配graphical buffer 
 * 
 * @param width 
 * @param height 
 * @param bpp  bits per pixel
 * @param buf 
 * @return int 0 on success,
 */
int hal_display_alloc_buffer(uint32_t width, uint32_t height, uint32_t bpp, hal_drm_buf_t* buf);

void hal_display_free_buffer(hal_drm_buf_t* buf);


void hal_display_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_DISPLAY_H