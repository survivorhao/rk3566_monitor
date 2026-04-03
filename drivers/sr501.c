/**
 * @file sr501.c    SR501人体红外传感器驱动
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



//最多10个SR501设备
#define MAX_SR501_DEVS   10

//主设备号
static int global_major; 

static struct class *sr501_class;

static int global_device_count;
static struct sr501_dev *g_devices[MAX_SR501_DEVS];



struct sr501_dev {

    //gpio descriptor
    struct gpio_desc* io;
    
    //virtual irq number
    int virq;

    //次设备号
    int minor;
    //主设备号
    int major;

    //等待队列头
    wait_queue_head_t wq;

    //1表示检测到人体红外线
    int motion_detected;
};


/**
 * @brief SR501中断处理函数上半部分，仅仅做一些很快的操作，因此不需要中断下半部分
 * 
 * @param irq virtual irq number
 * @param data user data
 * @return irqreturn_t  IRQ_HANDLED means success handle irq
 */
static irqreturn_t sr501_isr(int irq, void *data)
{   
    struct sr501_dev *dev = (struct sr501_dev*)data;
    
    /* 设置事件标志并唤醒等待队列 */
    dev->motion_detected = 1;
    wake_up_interruptible(&dev->wq);

    pr_debug("SR501 IRQ triggered \n");

    return IRQ_HANDLED;
}


/**
 * @brief 和设备结点匹配时候调用
 * 
 * @param pdev 
 * @return int 
 */
static int sr501_probe(struct platform_device *pdev)
{
    int ret;
    struct sr501_dev *pri_dev;
        
    if (global_device_count >= MAX_SR501_DEVS) return -ENOMEM;


    pri_dev = devm_kzalloc(&pdev->dev, sizeof(struct sr501_dev), GFP_KERNEL);
    if (!pri_dev) return -ENOMEM;
    
    platform_set_drvdata(pdev, pri_dev);

    pri_dev->minor = global_device_count;
    pri_dev->major = global_major;
    g_devices[pri_dev->minor] = pri_dev;

    /* 初始化等待队列头结点 */
    init_waitqueue_head(&pri_dev->wq);
    pri_dev->motion_detected = 0;

    device_create(sr501_class, NULL, MKDEV(pri_dev->major, pri_dev->minor), NULL,
                  "sr501_dev%d", pri_dev->minor);
    
    pri_dev->io = devm_gpiod_get(&pdev->dev, NULL, GPIOD_IN);                                           
    if (IS_ERR(pri_dev->io)) {
        dev_err(&pdev->dev, "get gpio property fail\n");
        ret = PTR_ERR(pri_dev->io);
        goto err_dev_create;
    }

    //gpio descriptor to virtual irq
    pri_dev->virq = gpiod_to_irq(pri_dev->io);
    if (pri_dev->virq < 0) {
        dev_err(&pdev->dev, "Failed to translate GPIO to IRQ\n");
        ret = pri_dev->virq;
        goto err_dev_create;
    }

    /* SR501默认输出低电平，检测到人体红外线输出高电平，这边我们配置上升沿触发 */
    ret = devm_request_irq(&pdev->dev, pri_dev->virq, sr501_isr, 
                           IRQF_TRIGGER_RISING,
                           "sr501_isr", pri_dev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to request IRQ\n");
        goto err_dev_create;
    }

    global_device_count++;
    dev_info(&pdev->dev, "gpio[%d]request irq[%d] success, major=%d, minor=%d\n",
    desc_to_gpio(pri_dev->io), pri_dev->virq, pri_dev->major, pri_dev->minor);

    return 0;

err_dev_create:
    device_destroy(sr501_class, MKDEV(pri_dev->major, pri_dev->minor));
    g_devices[pri_dev->minor] = NULL;
    return ret;
}

/**
 * @brief 驱动被删除的时候调用
 * 
 * @param pdev 
 * @return int 
 */
int sr501_remove(struct platform_device *pdev)
{
    struct sr501_dev* dev = platform_get_drvdata(pdev);

    //删除设备结点
    device_destroy(sr501_class, MKDEV(dev->major, dev->minor));
    g_devices[dev->minor] = NULL;
    global_device_count--;
   
    dev_info(&pdev->dev, "delete device: major=%d, minor=%d\n", dev->major, dev->minor);
    return 0;
}



static const struct of_device_id  sr501_of_match[] = {
        { .compatible = "my,sr501", },
        { },
};


static struct platform_driver sr501_driver = {
        .probe          = sr501_probe,
        .remove         = sr501_remove,
        .driver         = {
                .name   = "sr501_driver",
                .of_match_table = of_match_ptr(sr501_of_match)
        }
};


/**
 * @brief 
 * 
 * @param node 
 * @param file 
 * @return int 
 */
int sr501_open(struct inode *node, struct file *file)
{
    int minor = iminor(node);
    
    if (minor >= MAX_SR501_DEVS || !g_devices[minor]) {
        return -ENODEV;
    }

    /* 将当前设备的指针存入 file 的私有数据中，供 read/poll 使用 */
    file->private_data = g_devices[minor];
    pr_info("open sr501 device minor %d\n", minor);

    return 0;
}

/**
 * @brief 读SR501看是否检测到人体红外线
 * 
 * @param filp 
 * @param user_buf 
 * @param count 
 * @param ppos 
 * @return ssize_t return 2 on success
 */
static ssize_t sr501_read(struct file *filp, char __user *user_buf,
                          size_t count, loff_t *ppos)
{
    struct sr501_dev *dev = filp->private_data;
    char val_str[2];
    int ret;

    /* 1. 如果是非阻塞打开，且没有事件，直接返回 EAGAIN */
    if (filp->f_flags & O_NONBLOCK) {
        if (!dev->motion_detected)
            return -EAGAIN;
    } 
    else {
    
        /* 2. 阻塞等待：进程在这里休眠，直到 motion_detected 变为 1 且没有收到致命信号 */
        ret = wait_event_interruptible(dev->wq, dev->motion_detected);
        if (ret)
            return ret;     /* 被信号(如 Ctrl+C)打断 */
    }

    /* 3. 运行到此处表示已经检测到人体红外线，清除事件标志 */
    dev->motion_detected = 0;

    /* 4. 将状态返回给用户空间 (1 表示有人) */
    val_str[0] = 1;
    val_str[1] = '\n';

    if (count > 2) count = 2;
    if (copy_to_user(user_buf, val_str, count)) {
        return -EFAULT;
    }

    return count;
}

/**
 * @brief 提供给用户空间进行select/poll/epoll
 * 
 * @param filp 
 * @param wait 
 * @return __poll_t 
 */
static __poll_t sr501_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct sr501_dev *dev = filp->private_data;
    __poll_t mask = 0;

    poll_wait(filp, &dev->wq, wait);

    if (dev->motion_detected) {
        mask |= EPOLLIN | EPOLLPRI | EPOLLRDNORM;
    }
    return mask;
}

static struct file_operations sr501_chrdev_fops = {
    .owner   = THIS_MODULE,
    .open    = sr501_open,
    .read    = sr501_read,
    .poll    = sr501_poll,
};



/**
 * @brief insmod 时候调用
 * 
 * @return int 
 */
static int __init sr501_module_init(void)
{
    pr_info("begin init sr501 module ...\n");

    global_major=register_chrdev(0,"sr501_chrdev",&sr501_chrdev_fops);
    if (global_major < 0) 
    {
        pr_err("register chrdev fail ! \n");
        return -1;
    }

    sr501_class = class_create(THIS_MODULE, "sr501_class");

    //判断这个指针是否用来表示错误的，而不是一个指向有效内存地址的指针
    if (IS_ERR(sr501_class)) 
    {
        pr_err("create class fail !\n");
        unregister_chrdev(global_major, "my,platform_test_driver");
        return -1;
    }

    //注册平台驱动
    platform_driver_register(&sr501_driver); 
    
    pr_info("module init success \n");

    return 0;
}

/**
 * @brief rmmod 时候调用
 */
static void __exit sr501_module_exit(void)
{
    pr_info("sr501 module begin exit \n");

    //
    platform_driver_unregister(&sr501_driver);

    class_destroy(sr501_class);
    unregister_chrdev(global_major, "sr501_chrdev");
    
    pr_info("module exit success \n");

}

module_init(sr501_module_init);
module_exit(sr501_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sr501 device driver");