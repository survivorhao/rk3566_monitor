#ifndef SERVICE_STORAGE_H
#define SERVICE_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

// 初始化后台存储队列与压缩线程
int service_storage_init(void);

// 生产者接口：将 RGB 原图与传感器数据投入后台队列
int service_storage_push_task(const unsigned char *rgb_data, int width, int height,
                              int target_count, float temp, float humi, 
                              int co2, int tvoc);

// 销毁队列与线程
void service_storage_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SERVICE_STORAGE_H