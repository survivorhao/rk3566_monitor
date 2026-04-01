#include "hal_sensor.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <errno.h>
#include <sys/time.h>


// 默认看门狗时长为 10000 毫秒
static int g_watchdog_ms = 10000; 

// 内部隐藏的全局状态与锁
static struct {
    int co2;
    int tvoc;
    float temp;
    float humi;
    bool is_ai_active;       //是否需要激活ai进行检测
    uint64_t ai_timeout_ms;  //ai检测多少秒，没有目标，后进行休眠
} g_sensor_state = {0, 0, 0.0f, 0.0f, false, 0};

static pthread_mutex_t g_state_mutex = PTHREAD_MUTEX_INITIALIZER;

// 获取当前毫秒级时间戳
static uint64_t get_current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}


void hal_sensor_set_watchdog_duration(int ms) {
    pthread_mutex_lock(&g_state_mutex);
    g_watchdog_ms = ms;
    pthread_mutex_unlock(&g_state_mutex);
}

int hal_sensor_get_watchdog_duration(void) {
    return g_watchdog_ms;
}



/**
 * @brief 读取DHT11数据
 * 
 * @param path 
 * @return int 
 */
static int read_sysfs_int(const char *path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    char buf[32] = {0};
    read(fd, buf, sizeof(buf) - 1);
    close(fd);
    return atoi(buf);
}

/**
 * @brief 传感器数据读取线程，使用epoll来实现高效监听多个传感器
 * 
 * @param arg NULL,unused 
 * @return void* NULL,unused
 */
static void* sensor_thread_func(void* arg) 
{
    int epfd, sr501_fd, sgp30_fd;
    struct epoll_event ev, events[MAX_EPOLL_EVENTS];

    //非阻塞的方式打开设备结点
    sr501_fd = open(SR501_DEV, O_RDONLY | O_NONBLOCK);
    sgp30_fd = open(SGP30_DEV, O_RDONLY | O_NONBLOCK);
    
    if (sr501_fd < 0 || sgp30_fd < 0) {
        printf("[HAL Sensor] 警告：无法打开传感器节点！请检查驱动加载情况。\n");
        return NULL;
    }

    epfd = epoll_create1(0);
    ev.events = EPOLLIN; ev.data.fd = sr501_fd; epoll_ctl(epfd, EPOLL_CTL_ADD, sr501_fd, &ev);
    ev.events = EPOLLIN; ev.data.fd = sgp30_fd; epoll_ctl(epfd, EPOLL_CTL_ADD, sgp30_fd, &ev);

    printf("[HAL Sensor] 硬件传感器事件监听引擎已启动...\n");

    while (1) 
    {
        int nfds = epoll_wait(epfd, events, MAX_EPOLL_EVENTS, -1);
        if (nfds < 0) { if (errno == EINTR) continue; break; }

        for (int i = 0; i < nfds; ++i) {
            // --- SR501 人体红外触发 ---
            if (events[i].data.fd == sr501_fd) {
                char buf[4] = {0};
                if (read(sr501_fd, buf, sizeof(buf)) > 0) {
                    pthread_mutex_lock(&g_state_mutex);
                    g_sensor_state.is_ai_active = true;
                    g_sensor_state.ai_timeout_ms = get_current_time_ms() + g_watchdog_ms;
                    pthread_mutex_unlock(&g_state_mutex);
                    printf("\n [HAL Sensor] SR501 硬件中断触发，唤醒 AI 引擎 %d 秒！\n",g_watchdog_ms);
                }
            }
            // --- SGP30 1Hz 轮询触发 ---
            else if (events[i].data.fd == sgp30_fd) {
                int data[2] = {0};
                if (read(sgp30_fd, data, sizeof(data)) == sizeof(data)) {
                    
                    //读取DHT11温湿度数据
                    int t_raw = read_sysfs_int(DHT11_TEMP);
                    int h_raw = read_sysfs_int(DHT11_HUMI);
                    
                    pthread_mutex_lock(&g_state_mutex);
                    g_sensor_state.co2 = data[0];
                    g_sensor_state.tvoc = data[1];
                    g_sensor_state.temp = t_raw / 1000.0f;
                    g_sensor_state.humi = h_raw / 1000.0f;
                    pthread_mutex_unlock(&g_state_mutex);
                }
            }
        }
    }
    
    close(sr501_fd); close(sgp30_fd); close(epfd);
    return NULL;
}

/**
 * @brief   创建单独的线程来读取DHT11,SGP30,SR501传感器数据
 * 
 * @return int return 0 on success
 */
int hal_sensor_init(void) {
    pthread_t sensor_tid;
    if (pthread_create(&sensor_tid, NULL, sensor_thread_func, NULL) != 0) {
        return -1;
    }

    //设置为detached state,主线程无需手动回收资源
    pthread_detach(sensor_tid);
    return 0;
}

current_sensor_state_t hal_sensor_get_state(void) {
    current_sensor_state_t state;
    uint64_t now = get_current_time_ms();

    pthread_mutex_lock(&g_state_mutex);
    // 内部自动维护超时状态机的流转
    if (g_sensor_state.is_ai_active && now > g_sensor_state.ai_timeout_ms) {
        g_sensor_state.is_ai_active = false;
        printf("[HAL Sensor]  AI 唤醒超时 (已满10秒无动静)，进入低功耗休眠。\n");
    }
    
    state.co2 = g_sensor_state.co2;
    state.tvoc = g_sensor_state.tvoc;
    state.temp = g_sensor_state.temp;
    state.humi = g_sensor_state.humi;
    state.is_ai_active = g_sensor_state.is_ai_active;
    pthread_mutex_unlock(&g_state_mutex);

    return state;
}

void hal_sensor_keep_ai_alive(int duration_ms) {
    pthread_mutex_lock(&g_state_mutex);
    g_sensor_state.ai_timeout_ms = get_current_time_ms() + duration_ms;
    g_sensor_state.is_ai_active = true;
    pthread_mutex_unlock(&g_state_mutex);
}