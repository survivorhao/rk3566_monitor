#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// ==========================================
// 摄像头与 AI 视觉参数
// ==========================================
#define CAM_DEV "/dev/video0"
#define CAM_WIDTH  1280
#define CAM_HEIGHT 720
#define REQ_BUF_CNT 4

#define MODEL_WIDTH  640
#define MODEL_HEIGHT 640

#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 800

// ==========================================
// 传感器硬件节点配置
// ==========================================
#define SR501_DEV "/dev/sr501_dev0"
#define SGP30_DEV "/dev/sgp30_0"
#define DHT11_TEMP "/sys/bus/iio/devices/iio:device1/in_temp_input"
#define DHT11_HUMI "/sys/bus/iio/devices/iio:device1/in_humidityrelative_input"
#define MAX_EPOLL_EVENTS 5

// ==========================================
// MQTT 云端连接配置
// ==========================================
#define MQTT_HOST       "z9c1fa31.ala.cn-hangzhou.emqxsl.cn"
#define MQTT_PORT       8883
#define MQTT_USER       "rk3566dev"
#define MQTT_PWD        "2cpvhZLTX8UfX4R"
#define CA_CERT_PATH    "./emqxsl-ca.crt"
#define MQTT_TOPIC      "rk3566/ai_events"




// ==========================================
// 本地 SD 卡存储路径配置
// ==========================================
#define SAVE_BASE_DIR "/media/sdcard0/camera_log"
#define SAVE_IMG_DIR  "/media/sdcard0/camera_log/images"
#define SAVE_CSV_PATH "/media/sdcard0/camera_log/events_log.csv"

#endif // APP_CONFIG_H