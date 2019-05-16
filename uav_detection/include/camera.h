#ifndef __CAMERA_H
#define __CAMERA_H

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "djicam.h"

#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3


namespace camera{

struct sRGB{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class Camera
{
public:
    // Interface function
    bool init();
    bool capture(cv::Mat& img);
    void exit();
private:
    sRGB yuvTorgb(int Y, int U, int V);
    void NV12ToRGB(uint8_t* src, uint8_t* rgb, int width, int height);

private:
    uint8_t buffer[FRAME_SIZE];
    uint8_t pData[FRAME_SIZE];
    IplImage *pImg;
    uint32_t nframe;
    uint32_t block;
    int mode;
};

}

#endif // __CAMERA_H
