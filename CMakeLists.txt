cmake_minimum_required(VERSION 3.10)

# 编译应用程序需要先编译buildroot，这边使用buildroot生成的交叉编译器来编译应用程序
# 应用程序使用的库采用的buildroot生成的库
# 当然你不编译Buildroot也可以，自行指定交叉编译器以及库路径，可能会有兼容性问题
# 在命令中设置CMAKE_TOOLCHAIN_FILE变量指向你的buildroot工具链cmake脚本
# 无需手动设置交叉编译器gcc路径等
# 例如 
# cmake .. -D CMAKE_TOOCHAIN_FILE=/home/book/tspi_sdk/buildroot/output/rockchip_rk3566/host/share/buildroot/toolchainfile.cmake

project(smart_terminal)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# ---------------------------------------------------------
# 1. 头文件搜索路径配置
# ---------------------------------------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    #drm.h存放在sysroot/usr/include/libdrm/目录下
    ${CMAKE_SYSROOT}/usr/include/libdrm 
)

# ---------------------------------------------------------
# 2. 库的搜索路径配置
# ---------------------------------------------------------
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/vendor_lib)

# ---------------------------------------------------------
# 3. 收集源文件 & 生成目标
# ---------------------------------------------------------
file(GLOB_RECURSE SRC_FILES "src/*.cpp" "src/*.cc" "src/*.c")
add_executable(${PROJECT_NAME} ${SRC_FILES})

# ---------------------------------------------------------
# 4. 链接库
# ---------------------------------------------------------
target_link_libraries(${PROJECT_NAME}
    # ---- 以下 2 个库是从你的 vendor_lib 目录链接的 ----
    rknnrt      # RKNN NPU 核心推理库
    rga         # Rockchip 2D 图形硬件加速库

    # ---- 以下所有开源库，CMake 会自动去Sysroot里面找。
    turbojpeg   # JPEG 极速软件压缩库
    drm         # Linux 现代显示子系统接口库
    mosquitto   # MQTT 客户端库
    ssl         # OpenSSL 加密协议库
    crypto      # OpenSSL 密码学基础库
    m           # 标准 C 数学库
    pthread     # POSIX 线程库
    dl          # 动态加载库
)

# ---------------------------------------------------------
# 5. RPATH 设置 (运行时动态库搜索路径)
# ---------------------------------------------------------
# -Wl传递给链接器的命令
# 在板子上运行时，Linux 系统会自动去 /usr/lib 下找到开源库。
# 只有 rknnrt 和 rga 是板子系统里没有的。
# 这行代码的意思是：告诉程序，运行的时候去自己所在目录的 "lib" 文件夹下找那两个私有库。
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-rpath=$ORIGIN/lib")
