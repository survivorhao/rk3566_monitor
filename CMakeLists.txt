
cmake_minimum_required(VERSION 3.10)

# 【核心修复】：必须在 project() 之前告诉 CMake 我们的交叉编译器和目标架构！
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER "/home/book/tspi_sdk/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "/home/book/tspi_sdk/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++")

# 现在再声明工程名字，CMake 就会乖乖使用上面指定的 aarch64 编译器了
project(smart_terminal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 指定头文件搜索路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # 找到 xf86drm.h
    /home/book/tspi_sdk/external/camera_engine_rkaiq/tests/rkisp_demo/demo/include
    # 找到 drm.h 和 drm_fourcc.h
    /home/book/tspi_sdk/external/camera_engine_rkaiq/tests/rkisp_demo/demo/include/libdrm
)

# 指定动态库搜索路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 收集所有的 C/C++ 源文件 (现在加入了对 .c 文件的支持！)
file(GLOB SRC_FILES "src/*.cpp" "src/*.cc" "src/*.c")

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 链接核心引擎库
target_link_libraries(${PROJECT_NAME}
    rknnrt
    rga
    turbojpeg
    drm        # 现在它能在你的 lib/ 目录下找到 libdrm.so 了
    m          # <--- 【新增】链接标准数学库 (解决 sin/cos 报错)
    pthread
    dl
)
# 告诉链接器：在生成的可执行文件中写入 RPATH
# 让它在运行时，优先去自己同级目录下的 lib 文件夹找 .so 动态库
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,-rpath=$ORIGIN/lib")

