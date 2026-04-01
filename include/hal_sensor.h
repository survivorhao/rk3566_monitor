#ifndef HAL_SENSOR_H
#define HAL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 对外暴露的传感器与 AI 状态快照
typedef struct {
    int co2;
    int tvoc;
    float temp;
    float humi;
    bool is_ai_active; //是否需要激活ai进行检测
} current_sensor_state_t;

// 1. 初始化传感器模块并启动后台 epoll 监听线程
int hal_sensor_init(void);

// 2. 获取当前最新的环境数据与 AI 唤醒状态（内部自带互斥锁和超时判断）
current_sensor_state_t hal_sensor_get_state(void);

// 3. AI 视觉引擎调用：发现人体后，重置看门狗倒计时（例如传入 10000 毫秒）
void hal_sensor_keep_ai_alive(int duration_ms);

// 动态修改 AI 激活看门狗时长（毫秒）
void hal_sensor_set_watchdog_duration(int ms);
// 获取当前看门狗时长（毫秒）
int hal_sensor_get_watchdog_duration(void);


#ifdef __cplusplus
}
#endif

#endif // HAL_SENSOR_H