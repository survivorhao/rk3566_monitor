/**
 * @file sgp30.c    SGP30气体传感器驱动
 * @author kikyou
 * @brief 
 * @version 0.1
 * @date 2026-04-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mutex.h>

//最多10个SGP30设备
#define MAX_SGP30_DEVS   10

//主设备号
static int global_major; 
static struct class *sgp30_class;
static int global_device_count = 0;

struct sgp30_dev {
    struct i2c_client *client;
    struct device *dev_node;         // 用于 device_create 返回的设备节点
    
    //次设备号
    int minor;
    //主设备号
    int major;

    //等待队列 (用于 poll)
    wait_queue_head_t wq;

    // 延时工作队列与互斥锁
    struct delayed_work meas_work;

    struct mutex lock;

    // 传感器数据
    int tvoc_data;
    int co2_data;
    bool data_valid;    // 标记数据是否有效
    bool has_new_data;  // 标记是否有新数据供 poll 唤醒
};

static struct sgp30_dev *g_devices[MAX_SGP30_DEVS];

// 根据 SGP30 手册实现的 CRC-8 校验算法
static u8 sgp30_calc_crc(u8 data[2]) 
{
    u8 crc = 0xFF; // Initialization 0xFF
    int i, j;
    for (i = 0; i < 2; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31; // Polynomial 0x31
            else
                crc = (crc << 1);
        }
    }
    return crc;
}


/**
 * @brief 1Hz 的延时工作队列处理函数
 * 
 * @param work 
 */
static void sgp30_measurement_work(struct work_struct *work) 
{
    // 1.获取之前保存的设备结构体指针
    struct sgp30_dev *sgp = container_of(work, struct sgp30_dev, meas_work.work);
    u8 cmd[2] = {0x20, 0x08}; // sgp30_measure_iaq 命令
    u8 buf[6];
    int ret;

    // 2.发送测量命令
    ret = i2c_master_send(sgp->client, cmd, 2);
    if (ret < 0) {
        dev_err(&sgp->client->dev, "Failed to send measure command\n");
        goto reschedule;
    }

    // 3. 等待测量完成 (手册规定 Max 12ms)
    msleep(12);

    // 4. 读取 6 字节数据 (CO2 MSB, CO2 LSB, CRC, TVOC MSB, TVOC LSB, CRC)
    //                    0        1        2    3         4         5       
    ret = i2c_master_recv(sgp->client, buf, 6);
    if (ret == 6) 
    {
        // 5. 校验 CRC
        if (sgp30_calc_crc(&buf[0]) == buf[2] && sgp30_calc_crc(&buf[3]) == buf[5]) {
            mutex_lock(&sgp->lock);
            sgp->co2_data = (buf[0] << 8) | buf[1];
            sgp->tvoc_data = (buf[3] << 8) | buf[4];
            
            // 只要不是预热期的固定值(CO2=400, TVOC=0)，就认为数据有效
            if (sgp->co2_data != 400 || sgp->tvoc_data != 0) {
                sgp->data_valid = true;
            }
            
            //预留debug接口，直接输出数据
            dev_dbg(sgp->dev_node,"raw co2 is %d, tvoc is %d \n",sgp->co2_data, sgp->tvoc_data);

            sgp->has_new_data = true; // 标记有新数据
            mutex_unlock(&sgp->lock);
            
            // 6. 唤醒所有在 poll 里等待的进程！
            wake_up_interruptible(&sgp->wq);
        } else {
            dev_warn(&sgp->client->dev, "CRC checksum failed\n");
        }
    }

reschedule:
    // 6. 设定 1000 毫秒后再次执行自己，维持严格的 1Hz 心跳
    schedule_delayed_work(&sgp->meas_work, msecs_to_jiffies(1000));
}

/**
 * @brief  注册的i2c_driver和设备树中结点匹配的时候，调用这里的probe函数
 * 
 * @param client 内核已经帮我们转换好的一个i2c_client
 * @param id     已废弃，无需使用
 * @return int 
 */
static int sgp30_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct sgp30_dev *sgp; 
    u8 init_cmd[2] = {0x20, 0x03};          // sgp30_iaq_init 命令
    int ret;

    if (global_device_count >= MAX_SGP30_DEVS)
        return -ENOMEM;

    //1.分配私有驱动结构体
    sgp = devm_kzalloc(&client->dev, sizeof(struct sgp30_dev), GFP_KERNEL);
    if (!sgp) return -ENOMEM;

    sgp->client = client;
    sgp->major = global_major;
    sgp->minor = global_device_count;

    sgp->data_valid = false;
    
    sgp->has_new_data = false;

    // 2.初始化锁和等待队列
    mutex_init(&sgp->lock);
    init_waitqueue_head(&sgp->wq);


    //3.初始化delayed work，会在delay时间结束后，加入到内核的work queue中由内核线程执行
    INIT_DELAYED_WORK(&sgp->meas_work, sgp30_measurement_work);

    // 4.保存到全局数组
    g_devices[global_device_count] = sgp;

    // 5. 发送启动初始化命令，start datastop
    ret = i2c_master_send(client, init_cmd, 2);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to send init command\n");
        return ret;
    }
    
    // 6. 创建设备节点 /dev/sgp30_0
    sgp->dev_node = device_create(sgp30_class, &client->dev, 
                                  MKDEV(global_major, sgp->minor), 
                                  NULL, "sgp30_%d", sgp->minor);
    if (IS_ERR(sgp->dev_node)) {
        return PTR_ERR(sgp->dev_node);
    }

    // 7.把我们自定义的结构体设为 client 的私有数据
    i2c_set_clientdata(client, sgp);

    // 8. 启动后台引擎：1秒后执行第一次测算
    schedule_delayed_work(&sgp->meas_work, msecs_to_jiffies(1000));

    dev_dbg(&client->dev, "success probe sgp30(addr 0x%x, minor %d)\n",client->addr, sgp->minor);
             
    global_device_count++;
    return 0;
}

/**
 * @brief 当之前注册的驱动被i2c_del_driver,会调用这里的remove，
 *        解除已经建立的i2c_client和i2c_driver的绑定
 * 
 * @param client 
 * @return int 
 */
static int sgp30_remove(struct i2c_client *client)
{
    struct sgp30_dev *sgp = i2c_get_clientdata(client);

    // 1. 取消后台延时工作队列
    cancel_delayed_work_sync(&sgp->meas_work);

    // 2. 销毁设备节点
    device_destroy(sgp30_class, MKDEV(sgp->major, sgp->minor));
    
    g_devices[sgp->minor] = NULL;
    dev_dbg(&client->dev, "sgp30 removed\n");
    return 0;
}



static const struct of_device_id sgp30_of_match[] = {
        { .compatible = "my,sgp30", },
        { },
};


static struct i2c_driver sgp30_driver = {
        .probe          = sgp30_probe,
        .remove         = sgp30_remove,
        .driver         = {
                .name   = "sgp30_driver",
                .of_match_table = of_match_ptr(sgp30_of_match)
        }
};




int sgp30_open(struct inode *node, struct file *file)
{
    int minor = iminor(node);
    
    if (minor >= MAX_SGP30_DEVS || !g_devices[minor]) {
        return -ENODEV;
    }

    /* 将当前设备的指针存入 file 的私有数据中，供 read/poll 使用 */
    file->private_data = g_devices[minor];
    
    return 0;
}

/**
 * @brief 读取SGP30传感器，获得CO2以及TVOC数据
 * 
 * @param filp 
 * @param user_buf 
 * @param count 
 * @param ppos 
 * @return ssize_t 
 */
static ssize_t sgp30_read(struct file *filp, char __user *user_buf,
                          size_t count, loff_t *ppos)
{
    struct sgp30_dev *sgp = filp->private_data;
    int sensor_data[2];             // 在内核栈上分配一个局部数组
    
    // 1. 安全防线：检查用户空间提供的 buffer 够不够装 8 个字节
    if (count < sizeof(sensor_data)) {
        return -EINVAL; // 返回 Invalid argument (无效参数)
    }

    // 2. 极速锁定，提取数据
    mutex_lock(&sgp->lock);
    
    if (!sgp->data_valid) {
        mutex_unlock(&sgp->lock);
        pr_debug("SGP30 warming up, data not valid yet\n");
        return -ENODATA; // 返回 No data available (通知应用层：等会儿再来)
    } 

    // 把数据转移到内核栈的局部变量里
    sensor_data[0] = sgp->co2_data;
    sensor_data[1] = sgp->tvoc_data;
    sgp->has_new_data = false; // 清除新数据标志
    
    // 3.数据拿到手了，立刻解锁！绝对不让 copy_to_user 占用锁的时间！
    mutex_unlock(&sgp->lock);

    // 4. 在无锁状态下，一次性向用户空间拷贝
    // 如果 copy_to_user 返回非 0，说明有这么多字节没拷贝成功，证明用户态指针有问题
    if (copy_to_user(user_buf, sensor_data, sizeof(sensor_data))) {
        pr_err("copy_to_user failed\n");
        return -EFAULT;     // 返回 Bad address (坏的内存地址)
    }

    // 5. 返回成功读取的实际字节数！这才是标准驱动该返回的值。
    return sizeof(sensor_data); 
}

/**
 * @brief 提供给用户空间进行select/poll/epoll
 * 
 * @param filp 
 * @param wait 
 * @return __poll_t 
 */
static __poll_t sgp30_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct sgp30_dev *sgp = filp->private_data;
    __poll_t mask = 0;

    // 1. 把进程挂到等待队列上
    poll_wait(filp, &sgp->wq, wait);

    // 2. 检查是否有新数据
    mutex_lock(&sgp->lock);
    if (sgp->has_new_data) {
        mask |= EPOLLIN | EPOLLRDNORM; // 有数据可读
    }
    mutex_unlock(&sgp->lock);

    return mask;
}


//
static struct file_operations sgp30_chrdev_fops = {
    .owner   = THIS_MODULE,
    .open    = sgp30_open,
    .read    = sgp30_read,
    .poll    = sgp30_poll,
};

/**
 * @brief insmod 时候调用
 * 
 * @return int 
 */
static int __init sgp30_module_init(void)
{
    pr_info("begin init sgp30 driver module...\n");

    global_major = register_chrdev(0, "sgp30_chrdev", &sgp30_chrdev_fops);
    if (global_major < 0) {
        pr_err("register chrdev fail ! \n");
        return -1;
    }

    sgp30_class = class_create(THIS_MODULE, "sgp30_class");
    if (IS_ERR(sgp30_class)) {
        pr_err("create class fail !\n");
        unregister_chrdev(global_major, "sgp30_chrdev");
        return -1;
    }

    // 注册i2c driver
    i2c_add_driver(&sgp30_driver);
    pr_info("sgp30 driver module init success\n");
    return 0;
}



/**
 * @brief rmmod 时候调用
 */
static void __exit sgp30_module_exit(void)
{
    pr_info("sgp30 driver module begin exit \n");

    i2c_del_driver(&sgp30_driver);
    class_destroy(sgp30_class);
    unregister_chrdev(global_major, "sgp30_chrdev");
    
    pr_info("sgp30 driver module exit success\n");
}

module_init(sgp30_module_init);
module_exit(sgp30_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sgp30 device driver");