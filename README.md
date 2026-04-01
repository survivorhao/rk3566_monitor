# Smart Terminal

轻量级边缘智能感知终端，基于 RK3566（RK A55 + RKNPU）。

核心功能
- MIPI CSI 摄像头实时采集（OV5695）。
- 在板载 NPU 上运行 YOLOv5 进行人体检测。
- 多传感器融合：DHT11（温湿度，GPIO3_A3）、SGP30（CO2/TVOC，I2C）、HC-SR501（PIR，GPIO3_A4）。
- 事件落盘：检测到人体时将图像与传感器数据写入 SD 卡并通过 MQTT（TLS）上报。

快速上手
1. 交叉或本地构建：

   mkdir -p build && cd build && cmake ..
   make -j$(nproc)

2. 在目标板运行（需要 DRM 访问权限）：

   sudo ./build/smart_terminal <yolov5.rknn>

依赖（示例）: CMake, libdrm, libmosquitto, libssl, libturbojpeg

简要架构（高层）
- 主线程（`src/main.cpp`）：摄像头采集、RGA 预处理、渲染与触发 AI/Storage。
- AI 线程：后台运行 rknn 推理，结果回写共享结构，非阻塞显示路径。
- 传感器线程（`src/hal/hal_sensor.c`）：epoll 监听 PIR/SGP30，维护传感器状态与 AI 唤醒看门狗。
- 存储 Worker（`src/service/service_storage.c`）：工作队列 + 后台线程，负责 JPEG 压缩、SD 卡写入与 MQTT 上报。
- MQTT（`src/service/service_mqtt.c`）：使用 libmosquitto 异步线程处理网络与命令回调。

关键实现要点
- DRM 双缓冲与 page-flip（见 `src/hal/hal_display.c`），保证无撕裂显示。
- 使用 `drmPrimeHandleToFD` 导出 DMA fd，与 RGA 零拷贝协作做缩放/旋转。
- 主/AI/Storage 三条主路径分别处理实时显示、推理与 IO，避免互相阻塞。

代码位置（快速导航）
- 主程序: [src/main.cpp](src/main.cpp)
- 显示 HAL: [src/hal/hal_display.c](src/hal/hal_display.c)
- 摄像头 HAL: [src/hal/hal_camera.c](src/hal/hal_camera.c)
- 传感器 HAL: [src/hal/hal_sensor.c](src/hal/hal_sensor.c)
- Storage: [src/service/service_storage.c](src/service/service_storage.c)
- MQTT: [src/service/service_mqtt.c](src/service/service_mqtt.c)
- RKNPU + Yolov5: [src/rknpu/yolov5.cc](src/rknpu/yolov5.cc)

开源准备建议
- 添加 `LICENSE`（推荐 MIT）与 `.gitignore`（忽略 `build/`）。
- 可选：添加 `CONTRIBUTING.md` 与简单 CI（静态检查/交叉编译示例）。

欢迎提交 issue 与 PR，需讨论重要变更请先开 issue。

---

## 详细架构说明

本项目目标是在立创泰山派 RK3566 开发板上（Buildroot 根文件系统、Linux 4.19）构建一个集成 AI 视觉检测、多传感器融合、边缘存储与云同步的智能感知终端。下面针对各模块与运行时线程、数据流做具体说明，便于阅读代码与二次开发。

- 硬件平台
	- 主控：Rockchip RK3566（四核 Cortex-A55 + 1 TOPS NPU）
	- 摄像头：OV5695（MIPI CSI）
	- 温湿度：DHT11（GPIO: GPIO3_A3）
	- 气体传感器：SGP30（I2C）
	- 人体红外：HC-SR501（GPIO: GPIO3_A4）

- 功能概述
	1. 通过 MIPI CSI 摄像头实时采集画面。
	2. 在 NPU 上运行 YOLOv5 模型进行人体检测（rknn 接口）。
	3. 采集温湿度、CO2/TVOC、PIR 信号；当检测到人体事件时，将图像帧与传感器数据打包保存到 SD 卡，并通过 MQTT（TLS）上报到云端。

- 主要线程与运行时组件
	- 主线程（`src/main.cpp`）: 负责摄像头帧循环、使用 RGA 做图像预处理/缩放/旋转、调用 HAL 接口与发送给 AI 线程；同时与显示层（DRM）交互完成双缓冲提交。
	- AI 工作线程（`ai_worker_thread`，在 `src/main.cpp`）: 后台运行，等待主线程通过条件变量唤醒；调用 `inference_yolov5_model()` 在 RKNPU 上执行推理、过滤结果并将检测结果回写到共享变量供主线程渲染与触发抓拍。
	- 传感器线程（`sensor_thread_func`，在 `src/hal/hal_sensor.c`）: 使用 epoll 监听 SR501（PIR 中断）与 SGP30（周期性数据），并通过共享状态（加锁）向主线程提供最新环境数据与 AI 唤醒信号。
	- 存储 Worker（`storage_worker_thread`，在 `src/service/service_storage.c`）: 独立线程实现 work-queue，负责 JPEG 压缩（TurboJPEG）、本地落盘（图片 + CSV 记录）与通过 MQTT 上报。主线程仅向队列提交任务以降低实时路径开销。
	- MQTT 网络线程（由 libmosquitto 内部创建，见 `src/service/service_mqtt.c`）: 异步维护与 Broker 的连接、接收云端下发指令并回调主程序（例如修改置信度、强制抓拍、清理存储）。

- 关键同步与缓冲机制
	- DRM 双缓冲（`src/hal/hal_display.c`）: 使用两个 dumb buffer，当前 front/back 通过 `drmModePageFlip()` 与 page-flip 事件完成无撕裂切换； `hal_display_commit_and_wait()` 会等待 page-flip 完成后交换索引。
	- RGA 零拷贝：使用 `drmPrimeHandleToFD` 导出 dma fd，RGA 以 fd 作为输入/输出进行硬件拷贝/缩放/旋转，减少 CPU 内存拷贝。
	- AI 线程通信：主线程与 AI 线程通过 `pthread_mutex_t` + `pthread_cond_t` 协同，主线程喂入一帧并 `pthread_cond_signal()` 唤醒 AI，AI 完成后更新共享结果并设置触发抓拍标志。
	- 存储队列：`service_storage_push_task()` 将任务放入链表队列并用条件变量唤醒 Storage Worker，Worker 在后台异步压缩与上传，防止主循环阻塞。

- 数据流（从采集到上报）
	1. 摄像头（V4L2/MIPI）采集 NV12 帧。
	2. 主线程用 RGA 将 NV12 快速转换为展示用 RGB canvas（零拷贝），并用 RGA 将缩放后的 RGB 数据准备给模型输入缓冲（`MODEL_WIDTH`/`MODEL_HEIGHT`）。
	3. 主线程通过条件变量通知 AI 工作线程进行推理，AI 在 NPU 完成后写回检测结果。
	4. 若检测到目标或传感器触发（PIR），主线程把当前 canvas 的 RGB 数据深拷贝并调用 `service_storage_push_task()`，由 Storage Worker 完成 JPEG 压缩、本地写盘与 MQTT 上报。

传感器数据 -> `hal_sensor` 的 epoll 线程更新内部状态；当 PIR 触发或 AI 检测到人时，传感器值与帧一起保存并上报。

## 与 Buildroot / 内核的集成注意点

- 需要在 Buildroot 配置中启用对应的摄像头驱动（V4L2 / MIPI CSI）、RK 平台的 DRM 驱动与 RGA 驱动，以及 I2C、GPIO 接口驱动（用于 DHT11/HC-SR501/SGP30）。
- 确保内核启用了 `drm`、`drm_kms_helper`、`drm/rockchip` 与 `v4l2` 相关选项（kernel v4.19 在 RK3566 常见）。
- 将 CA 证书与 Mosquitto TLS 配置部署到设备，以便 `service_mqtt` 使用 TLS 安全连接云端 Broker。

## 位置与文件参考

- 主程序: [src/main.cpp](src/main.cpp)
- 显示 HAL: [src/hal/hal_display.c](src/hal/hal_display.c)
- 摄像头 HAL: [src/hal/hal_camera.c](src/hal/hal_camera.c)
- 传感器 HAL: [src/hal/hal_sensor.c](src/hal/hal_sensor.c)
- Storage: [src/service/service_storage.c](src/service/service_storage.c)
- MQTT: [src/service/service_mqtt.c](src/service/service_mqtt.c)
- RKNPU + Yolov5: [src/rknpu/yolov5.cc](src/rknpu/yolov5.cc)



