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



/**
 * @brief This is called when the broker has received the
 *        DISCONNECT command and has disconnected the client
 * 
 * @param mosq  the mosquitto instance making the callback.
 * @param obj   the user data provided in <mosquitto_new>
 * @param rc    integer value indicating the reason for the disconnect. A value of 0
 *         means the client has called <mosquitto_disconnect>. Any other value
 *         indicates that the disconnect is unexpected.
 */
static void on_disconnect(struct mosquitto *mosq, void *obj, int rc) {
    printf("[MQTT Service] warnning: connection witch broker terminated (return code: %d)! wait reconnecting...\n", rc);
    g_mqtt_connected = false;
}



bool is_mqtt_connected(void) 
{
    return g_mqtt_connected;
}


static mqtt_cmd_cb_t g_cmd_cb = NULL;


void mqtt_set_cmd_callback(mqtt_cmd_cb_t cb) {
    g_cmd_cb = cb;
}

/**
 * @brief 注册的mqtt connect call back function
 * 
 * @param mosq mqtt client instance
 * @param obj user pass argument
 * @param rc return code 
 */
static void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) 
    {
        printf("[MQTT Service] success connect to EMQX Broker!\n");
        g_mqtt_connected = true;

        // 连上后立即订阅 CMD 主题 (QoS 1),实现反向控制
        mosquitto_subscribe(mosq, NULL, MQTT_CMD_TOPIC, 1);
    } 
    else 
    {
        printf("[MQTT Service] connect fail,error code: %d\n", rc);
        g_mqtt_connected = false;
    }
}

/**
 * @brief This is called when a message is received from the
 *        broker and the required QoS flow has completed.
 * 
 * @param mosq mqtt client instance
 * @param obj user pass argument
 * @param msg the message data. This variable and associated memory will be
 *            freed by the library after the callback completes. The client
 *            should make copies of any of the data it requires.
 */
static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    if (!msg->payload || !g_cmd_cb) return;
    char *payload = (char*)msg->payload;

    // 简易且极速的 JSON 解析 (避免引入庞大的 cJSON 库)
    float val = 0.0f;
    char *val_ptr = strstr(payload, "\"val\"");
    if (val_ptr) {
        val_ptr += 5; // 跳过 "val"
        while(*val_ptr == ' ' || *val_ptr == ':') val_ptr++; // 跳过冒号和空格
        val = atof(val_ptr);
    }

    // 路由分发指令
    if (strstr(payload, "\"set_timeout\"")) g_cmd_cb("set_timeout", val);
    else if (strstr(payload, "\"set_threshold\"")) g_cmd_cb("set_threshold", val);
    else if (strstr(payload, "\"snapshot\"")) g_cmd_cb("snapshot", 0);
    else if (strstr(payload, "\"clear_storage\"")) g_cmd_cb("clear_storage", 0);


}




/**
 * @brief 向指定的topic上传数据
 * 
 * @param target_count 检测到几个目标（也就是检测到几个人）
 * @param temp 温度数据
 * @param humi 湿度数据
 * @param co2 二氧化碳浓度
 * @param tvoc 挥发性有机化合物
 * @param jpeg_buf jpeg格式图片数据
 * @param jpeg_size  大小
 * @return int return 0 on success
 */
int mqtt_report_event(int target_count, float temp, float humi, int co2, int tvoc, 
                      const unsigned char *jpeg_buf, unsigned long jpeg_size) {
    
    if (!mqtt_client || !g_mqtt_connected) return -1;

    // 1. OpenSSL Base64 编码
    size_t b64_max_len = 4 * ((jpeg_size + 2) / 3) + 1;

    // 2.分配buffer，存放base64编码后的Image data
    char *b64_buf = (char *)malloc(b64_max_len);
    if (!b64_buf) return -1;
    
    //对原始jepg图像数据进行base64编码
    int b64_len = EVP_EncodeBlock((unsigned char *)b64_buf, jpeg_buf, jpeg_size);

    // 3. 组装 JSON
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

    // 4. 发布消息
    int pub_ret = mosquitto_publish(mqtt_client, NULL, MQTT_TOPIC, strlen(json_buf), json_buf, 0, false);
    
    if (pub_ret == MOSQ_ERR_SUCCESS) {
        printf(">> [MQTT Service] success publish data! (JPG Image: %lu bytes)\n", jpeg_size);
    } else {
        printf(">> [MQTT Service Error] publish fail ! error code:%d\n", pub_ret);
    }

    free(json_buf); 
    free(b64_buf);

    return (pub_ret == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

/**
 * @brief 初始化mqtt client
 * 
 * @return int reuturn 0 on success
 */
int mqtt_init(void) 
{
    //init mosquitto lib
    mosquitto_lib_init();


    mqtt_client = mosquitto_new(MQTT_CLIENT_NAME, true, NULL);
    if (!mqtt_client) return -1;

    mosquitto_connect_callback_set(mqtt_client, on_connect);
    mosquitto_disconnect_callback_set(mqtt_client, on_disconnect);
    mosquitto_username_pw_set(mqtt_client, MQTT_USER, MQTT_PWD);
    
    // TLS 配置
    int tls_rc = mosquitto_tls_set(mqtt_client, CA_CERT_PATH, NULL, NULL, NULL, NULL);
    if (tls_rc != MOSQ_ERR_SUCCESS) {
        printf("[MQTT Service] TLS CA load fail: %s,please whether have CA file \n", mosquitto_strerror(tls_rc));
        return -1;
    }

    // 注册这个 message 回调
    mosquitto_message_callback_set(mqtt_client, on_message);

    //async connect mqtt broker
    mosquitto_connect_async(mqtt_client, MQTT_HOST, MQTT_PORT, 60);
    
    //start a new thread to process network traffic
    mosquitto_loop_start(mqtt_client);
    
    return 0;
}



/**
 * @brief free resources
 * 
 */
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