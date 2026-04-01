cmake_minimum_required(VERSION 3.10)

#指定最终程序运行的操作系统以及架构
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 设置c and c++编译器路径
set(CMAKE_C_COMPILER "/home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/bin/aarch64-buildroot-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/bin/aarch64-buildroot-linux-gnu-g++")

project(smart_terminal)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 指定头文件搜索路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/aarch64-buildroot-linux-gnu/sysroot/usr/include/libdrm
)

# 指定库的搜索路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 搜集所有源文件
file(GLOB_RECURSE SRC_FILES "src/*.cpp" "src/*.cc" "src/*.c")

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 链接核心引擎库
target_link_libraries(${PROJECT_NAME}
    rknnrt      # RKNN NPU 核心推理库
    rga         # Rockchip 2D 图形硬件加速库
    turbojpeg   # 静态链接的 JPEG 极速软件压缩库
    drm         # Linux 现代显示子系统接口库
    m           # 标准 C 数学库 (Math)
    pthread     # POSIX 线程库，供我们的异步多线程架构使用
    dl          # 动态加载库 (Dynamic Loader)
                # mosquitto 依赖 ssl 和 crypto
    mosquitto   # MQTT 客户端库
    ssl         # OpenSSL 加密协议库 (用于 TLS 连接 EMQX)
    crypto      # OpenSSL 密码学基础库
)

# 告诉链接器：在生成的可执行文件中写入 RPATH,
# $ORIGIN 表示 ELF 文件自身所在目录，路径会自动相对解析，非常便携
# 去可执行文件所在路径下的lib目录下查找
# rpath run-time search path运行时搜索路径 
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-rpath=$ORIGIN/lib")

