#ifndef HAL_CAMERA_H
#define HAL_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

// 1. 初始化 V4L2 摄像头，分配 DMA 内存，开启流
int hal_camera_init(void);

// 2. 从队列拿取一帧画面
// 输出：out_dma_fd (物理内存句柄供 RGA 使用), out_index (内部队列索引)
int hal_camera_get_frame(int *out_dma_fd, int *out_index);

// 3. 用完画面后，将其还给摄像头驱动队列
int hal_camera_put_frame(int index);

// 4. 关闭摄像头，释放资源
void hal_camera_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_CAMERA_H