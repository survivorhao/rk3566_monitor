#include "service_storage.h"
#include "app_config.h"
#include "service_mqtt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <turbojpeg.h>

// 任务节点 (RGB 原图与环境快照)
typedef struct task_node {
    unsigned char *rgb_data;
    int width;
    int height;
    int target_count;
    float temp;
    float humi;
    int co2;
    int tvoc;
    struct timeval timestamp;
    struct task_node *next;
} task_node_t;

// 阻塞队列结构体
typedef struct {
    task_node_t *head;
    task_node_t *tail;
    int count;
    bool stop_flag;
    pthread_mutex_t lock;
    pthread_cond_t cond;
} task_queue_t;

static task_queue_t g_queue;
static pthread_t g_worker_tid;

// 递归创建目录
static void ensure_dir_exists(const char *dir_path) {
    struct stat st = {0};
    if (stat(dir_path, &st) == -1) {
        mkdir(dir_path, 0777);
    }
}

// 消费者 Worker 线程
static void* storage_worker_thread(void* arg) 
{
    tjhandle tj_compressor = tjInitCompress();
    
    // 初始化时确保目录存在
    ensure_dir_exists(SAVE_BASE_DIR);
    ensure_dir_exists(SAVE_IMG_DIR);

    while (1) 
    {
        task_node_t *task = NULL;

        pthread_mutex_lock(&g_queue.lock);
        // 如果队列为空且没有收到停止信号，就休眠等待
        while (g_queue.count == 0 && !g_queue.stop_flag) {
            pthread_cond_wait(&g_queue.cond, &g_queue.lock);
        }

        // 如果收到停止信号且队列处理完了，退出线程
        if (g_queue.stop_flag && g_queue.count == 0) {
            pthread_mutex_unlock(&g_queue.lock);
            break;
        }

        // 出队
        task = g_queue.head;
        if (task) {
            g_queue.head = task->next;
            if (g_queue.head == NULL) g_queue.tail = NULL;
            g_queue.count--;
        }
        pthread_mutex_unlock(&g_queue.lock);

        if (task) 
        {
            // 1. 压缩为 JPEG
            unsigned char *jpeg_buf = NULL;
            unsigned long jpeg_size = 0;
            if (tjCompress2(tj_compressor, task->rgb_data, task->width, 0, task->height, 
                            TJPF_RGB, &jpeg_buf, &jpeg_size, TJSAMP_422, 75, TJFLAG_FASTDCT) == 0) {
                
                // 格式化时间戳
                struct tm *timeinfo = localtime(&task->timestamp.tv_sec);
                char time_str[64];
                strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", timeinfo);
                
                char img_path[256];
                snprintf(img_path, sizeof(img_path), "%s/cap_%s_tgt%d.jpg", SAVE_IMG_DIR, time_str, task->target_count);

                // 2. 本地落盘: 写入 JPEG 图片
                FILE *fp_img = fopen(img_path, "wb");
                if (fp_img) {
                    fwrite(jpeg_buf, 1, jpeg_size, fp_img);
                    fclose(fp_img);
                }

                // 3. 本地落盘: 追加 CSV 记录
                FILE *fp_csv = fopen(SAVE_CSV_PATH, "a");
                if (fp_csv) {
                    // 如果文件为空，先写个表头
                    fseek(fp_csv, 0, SEEK_END);
                    if (ftell(fp_csv) == 0) {
                        fprintf(fp_csv, "Timestamp,Datetime,TargetCount,Temp(C),Humi(%%),CO2(ppm),TVOC(ppb),ImagePath\n");
                    }
                    fprintf(fp_csv, "%ld,%s,%d,%.1f,%.1f,%d,%d,%s\n", 
                            task->timestamp.tv_sec, time_str, task->target_count, 
                            task->temp, task->humi, task->co2, task->tvoc, img_path);
                    fclose(fp_csv);
                }

                // 4. 云端同步: 发送 MQTT
                if (is_mqtt_connected()) {
                    mqtt_report_event(task->target_count, task->temp, task->humi, 
                                      task->co2, task->tvoc, jpeg_buf, jpeg_size);
                } else {
                    printf("[Storage Worker] 网络离线，数据仅保存在本地 SD 卡。\n");
                }

                tjFree(jpeg_buf);
            } else {
                printf("[Storage Worker Error] JPEG 压缩失败: %s\n", tjGetErrorStr());
            }

            // 5. 释放任务节点的内存（极度重要，防止内存泄漏）
            free(task->rgb_data);
            free(task);
        }
    }

    tjDestroy(tj_compressor);
    return NULL;
}

int service_storage_init(void) {
    memset(&g_queue, 0, sizeof(g_queue));
    pthread_mutex_init(&g_queue.lock, NULL);
    pthread_cond_init(&g_queue.cond, NULL);

    if (pthread_create(&g_worker_tid, NULL, storage_worker_thread, NULL) != 0) {
        printf("创建 Storage Worker 线程失败！\n");
        return -1;
    }
    return 0;
}

int service_storage_push_task(const unsigned char *rgb_data, int width, int height,
                              int target_count, float temp, float humi, int co2, int tvoc) {
    
    // 防爆破机制：如果队列堆积超过 10 帧来不及处理，主动丢弃，保护内存
    pthread_mutex_lock(&g_queue.lock);
    if (g_queue.count >= 10) {
        pthread_mutex_unlock(&g_queue.lock);
        return -1; // 队列满，抛弃该帧
    }
    pthread_mutex_unlock(&g_queue.lock);

    // 申请堆内存并深拷贝画面 (策略 A：主线程极速剥离)
    task_node_t *new_node = (task_node_t *)malloc(sizeof(task_node_t));
    if (!new_node) return -1;
    
    int rgb_size = width * height * 3;
    new_node->rgb_data = (unsigned char *)malloc(rgb_size);
    if (!new_node->rgb_data) {
        free(new_node);
        return -1;
    }

    //拷贝drm dumb buffer中的数据，到这里的malloc分配的buffer中
    //后续仅仅做image压缩到jpeg操作因此对于存放image的缓冲区的特性没有什么要求
    //例如物理内存是否连续，是否支持dma
    memcpy(new_node->rgb_data, rgb_data, rgb_size);

    new_node->width = width;
    new_node->height = height;
    new_node->target_count = target_count;
    new_node->temp = temp;
    new_node->humi = humi;
    new_node->co2 = co2;
    new_node->tvoc = tvoc;
    gettimeofday(&new_node->timestamp, NULL);
    new_node->next = NULL;

    // 加锁入队并唤醒 Worker
    pthread_mutex_lock(&g_queue.lock);
    if (g_queue.tail) {
        g_queue.tail->next = new_node;
    } else {
        g_queue.head = new_node;
    }
    g_queue.tail = new_node;
    g_queue.count++;
    pthread_cond_signal(&g_queue.cond); // 摇铃铛
    pthread_mutex_unlock(&g_queue.lock);

    return 0;
}

void service_storage_deinit(void) {
    pthread_mutex_lock(&g_queue.lock);
    g_queue.stop_flag = true;
    pthread_cond_broadcast(&g_queue.cond);
    pthread_mutex_unlock(&g_queue.lock);

    pthread_join(g_worker_tid, NULL);

    pthread_mutex_destroy(&g_queue.lock);
    pthread_cond_destroy(&g_queue.cond);
}