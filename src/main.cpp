// 文件路径: ~/smart_terminal/src/main.cpp
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// #include "rknn_api.h"      // NPU 头文件
// #include "RgaApi.h"        // RGA 头文件

// 【核心修复】：必须先包含 yolov5.h，因为里面定义了 context 结构体
#include "yolov5.h"        

// 然后再包含 postprocess.h
#include "postprocess.h"   



int main(int argc,char **argv) 
{
    printf("hello rknn! I get it !\n");
    

    int ret;
    const char *model_path = argv[1];

    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    ret = init_yolov5_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        return -1;
    }

    // 获取 RKNN SDK 版本信息作为测试
    rknn_sdk_version version;
    ret = rknn_query(rknn_app_ctx.rknn_ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret == RKNN_SUCC) {
        printf("current SDK version: %s,driver verison:%s \n",version.api_version,version.drv_version);

    }
    
    return 0;
}
