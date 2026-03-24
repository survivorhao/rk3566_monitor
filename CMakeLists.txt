cmake_minimum_required(VERSION 3.10)

# 【核心修复】：换用 Buildroot 生成的配套现代版交叉编译器！
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 注意：编译器位于 host/bin 目录下，而不是 host/aarch64-buildroot-linux-gnu/bin 目录
set(CMAKE_C_COMPILER "/home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/bin/aarch64-buildroot-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/bin/aarch64-buildroot-linux-gnu-g++")

project(smart_terminal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 指定头文件搜索路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /home/book/tspi_sdk/external/camera_engine_rkaiq/tests/rkisp_demo/demo/include
    /home/book/tspi_sdk/external/camera_engine_rkaiq/tests/rkisp_demo/demo/include/libdrm
    # mosquitto 头文件路径
    /home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/aarch64-buildroot-linux-gnu/sysroot/usr/include
)

# 指定动态库搜索路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 收集所有的 C/C++ 源文件
file(GLOB SRC_FILES "src/*.cpp" "src/*.cc" "src/*.c")

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 链接核心引擎库
target_link_libraries(${PROJECT_NAME}
    rknnrt
    rga
    turbojpeg
    drm        
    m          
    pthread
    dl
    # 【新增】顺序很重要！mosquitto 依赖 ssl 和 crypto
    mosquitto
    ssl
    crypto
)

# 告诉链接器：在生成的可执行文件中写入 RPATH
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-rpath=$ORIGIN/lib")

