#include "hal_camera.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

struct CamBuffer {
    void* start;
    size_t length;
    int dma_fd;
};

static struct CamBuffer* buffers = NULL;
static int cam_fd = -1;

int hal_camera_init(void) {
    cam_fd = open(CAM_DEV, O_RDWR);
    if (cam_fd < 0) {
        printf("[HAL Camera] Error: 打开摄像头设备失败\n");
        return -1;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = CAM_WIDTH;
    fmt.fmt.pix_mp.height = CAM_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12; 
    if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) < 0) return -1;

    struct v4l2_requestbuffers req = {0};
    req.count = REQ_BUF_CNT; 
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; 
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(cam_fd, VIDIOC_REQBUFS, &req) < 0) return -1;

    buffers = (struct CamBuffer*)calloc(req.count, sizeof(struct CamBuffer));
    for (int i = 0; i < req.count; ++i) {
        struct v4l2_plane planes[1] = {0}; 
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; 
        buf.memory = V4L2_MEMORY_MMAP; 
        buf.index = i; 
        buf.m.planes = planes; 
        buf.length = 1;
        ioctl(cam_fd, VIDIOC_QUERYBUF, &buf);
        
        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buf.m.planes[0].m.mem_offset);
        
        struct v4l2_exportbuffer expbuf = {0}; 
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; 
        expbuf.index = i;
        ioctl(cam_fd, VIDIOC_EXPBUF, &expbuf); 
        buffers[i].dma_fd = expbuf.fd;
        
        ioctl(cam_fd, VIDIOC_QBUF, &buf);
    }
    
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &type) < 0) return -1;

    printf("[HAL Camera] 摄像头 V4L2 引擎启动成功 (DMA 直通开启)\n");
    return 0;
}

int hal_camera_get_frame(int *out_dma_fd, int *out_index) {
    struct v4l2_plane planes[1] = {0}; 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; 
    buf.memory = V4L2_MEMORY_MMAP; 
    buf.m.planes = planes; 
    buf.length = 1;

    if (ioctl(cam_fd, VIDIOC_DQBUF, &buf) < 0) return -1;

    *out_index = buf.index;
    *out_dma_fd = buffers[buf.index].dma_fd;
    return 0;
}

int hal_camera_put_frame(int index) {
    struct v4l2_plane planes[1] = {0}; 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; 
    buf.memory = V4L2_MEMORY_MMAP; 
    buf.m.planes = planes; 
    buf.length = 1;
    buf.index = index;

    return ioctl(cam_fd, VIDIOC_QBUF, &buf);
}

void hal_camera_deinit(void) {
    if (cam_fd < 0) return;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
    
    if (buffers) {
        for (int i = 0; i < REQ_BUF_CNT; ++i) {
            if (buffers[i].dma_fd > 0) close(buffers[i].dma_fd); 
            if (buffers[i].start != NULL && buffers[i].start != MAP_FAILED) 
                munmap(buffers[i].start, buffers[i].length);
        }
        free(buffers);
    }
    
    struct v4l2_requestbuffers req_free = {0};
    req_free.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req_free.memory = V4L2_MEMORY_MMAP;
    ioctl(cam_fd, VIDIOC_REQBUFS, &req_free);

    close(cam_fd);
    cam_fd = -1;
}