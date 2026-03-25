#include "service_mqtt.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <mosquitto.h>
#include <openssl/evp.h>

static struct mosquitto *mqtt_client = NULL;
static bool g_mqtt_connected = false;

// MQTT 连接成功回调
static void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        printf("[MQTT Service] 成功通过 TLS 连接到 EMQX Broker!\n");
        g_mqtt_connected = true;
    } else {
        printf("[MQTT Service] 连接失败，错误码: %d\n", rc);
        g_mqtt_connected = false;
    }
}

// MQTT 断开回调
static void on_disconnect(struct mosquitto *mosq, void *obj, int rc) {
    printf("[MQTT Service] 警告：与服务器的连接已断开 (原因码: %d)！等待重连...\n", rc);
    g_mqtt_connected = false;
}

int mqtt_init(void) 
{
    mosquitto_lib_init();
    mqtt_client = mosquitto_new("rk3566_ai_terminal", true, NULL);
    if (!mqtt_client) return -1;

    mosquitto_connect_callback_set(mqtt_client, on_connect);
    mosquitto_disconnect_callback_set(mqtt_client, on_disconnect);
    mosquitto_username_pw_set(mqtt_client, MQTT_USER, MQTT_PWD);
    
    // TLS 配置
    int tls_rc = mosquitto_tls_set(mqtt_client, CA_CERT_PATH, NULL, NULL, NULL, NULL);
    if (tls_rc != MOSQ_ERR_SUCCESS) {
        printf("[MQTT Service] TLS 证书加载失败: %s\n", mosquitto_strerror(tls_rc));
        return -1;
    }

    mosquitto_connect_async(mqtt_client, MQTT_HOST, MQTT_PORT, 60);
    mosquitto_loop_start(mqtt_client);
    
    return 0;
}

bool is_mqtt_connected(void) 
{
    return g_mqtt_connected;
}

int mqtt_report_event(int target_count, float temp, float humi, int co2, int tvoc, 
                      const unsigned char *jpeg_buf, unsigned long jpeg_size) {
    
    if (!mqtt_client || !g_mqtt_connected) return -1;

    // 1. OpenSSL Base64 编码
    size_t b64_max_len = 4 * ((jpeg_size + 2) / 3) + 1;
    char *b64_buf = (char *)malloc(b64_max_len);
    if (!b64_buf) return -1;
    
    int b64_len = EVP_EncodeBlock((unsigned char *)b64_buf, jpeg_buf, jpeg_size);

    // 2. 组装 JSON
    size_t json_size = b64_len + 512; 
    char *json_buf = (char *)malloc(json_size);
    if (!json_buf) {
        free(b64_buf);
        return -1;
    }

    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    snprintf(json_buf, json_size, 
             "{\"timestamp\": %ld, \"target_count\": %d, \"temp\": %.1f, \"humi\": %.1f, \"co2\": %d, \"tvoc\": %d, \"image\": \"data:image/jpeg;base64,%s\"}", 
             current_time.tv_sec, target_count, temp, humi, co2, tvoc, b64_buf);

    // 3. 发布消息
    int pub_ret = mosquitto_publish(mqtt_client, NULL, MQTT_TOPIC, strlen(json_buf), json_buf, 0, false);
    
    if (pub_ret == MOSQ_ERR_SUCCESS) {
        printf(">> [MQTT Service] 已成功上报事件、传感器数据与抓拍图! (JPG: %lu bytes)\n", jpeg_size);
    } else {
        printf(">> [MQTT Service Error] 发送失败！错误码：%d\n", pub_ret);
    }

    free(json_buf); 
    free(b64_buf);

    return (pub_ret == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

void mqtt_cleanup(void) {
    if (mqtt_client) {
        mosquitto_disconnect(mqtt_client);
        mosquitto_loop_stop(mqtt_client, true);
        mosquitto_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    mosquitto_lib_cleanup();
    g_mqtt_connected = false;
}