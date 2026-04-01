#ifndef SERVICE_STORAGE_H
#define SERVICE_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建单独的线程，来将传感器以及图像数据存储到本地sd卡
 *        采用work queue的形式，主线程只需要向这个work queue递交任务
 *        这个线程负责在后台把数据保存到sd卡以及上传到Mqtt Broker
 * 
 * @return int return 0 on success
 */
int service_storage_init(void);


/**
 * @brief 向work queue递交一个task
 * 
 * @param rgb_data  要上传的图像帧数据
 * @param width     图像帧数据的宽度
 * @param height    图像帧数据的高度
 * @param target_count  检测到几个目标
 * @param temp 温度
 * @param humi 湿度
 * @param co2 二氧化碳
 * @param tvoc 总挥发性有机物
 * @return int return 0 on success, -1 on fail
 */
int service_storage_push_task(const unsigned char *rgb_data, int width, int height,
                              int target_count, float temp, float humi, 
                              int co2, int tvoc);

// 销毁队列与线程
void service_storage_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SERVICE_STORAGE_H