# 存放已经编译好的可直接使用的文件
- rootfs.img buildroot跟文件系统，添加了MQTT库以及TLS加密相关
- boot.img 内核以及dtb（使用官方的linux kernel 4.19）
- sgp30.ko sgp30驱动，使用sudo insmod加载到内核
- sr501.ko sr501驱动


