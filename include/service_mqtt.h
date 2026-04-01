#ifndef SERVICE_MQTT_H
#define SERVICE_MQTT_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// 1. 初始化 MQTT 客户端并建立异步连接
int mqtt_init(void);

// 2. 是否成功和Mqtt Broker建立连接
bool is_mqtt_connected(void);

// 3. 将抓拍图与环境数据打包并发送到云端
int mqtt_report_event(int target_count, float temp, float humi, int co2, int tvoc, 
                      const unsigned char *jpeg_buf, unsigned long jpeg_size);

// 4. 清理并断开连接
void mqtt_cleanup(void);

// 定义云端指令回调函数的函数指针类型
typedef void (*mqtt_cmd_cb_t)(const char* cmd, float val);

// 注册指令回调函数
void mqtt_set_cmd_callback(mqtt_cmd_cb_t cb);


#ifdef __cplusplus
}
#endif

#endif // SERVICE_MQTT_H