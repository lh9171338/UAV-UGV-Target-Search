#include "camera.h"


namespace camera{

bool Camera::init()
{
    // Initial object
    pImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H), IPL_DEPTH_8U, 3);
    nframe = 0;
    block = 1;
    mode = GETBUFFER_MODE | TRANSFER_MODE;

    // Initial camera
    int ret = manifold_cam_init(mode);
    if(ret == -1)
    {
        return false;
    }
    return true;
}

bool Camera::capture(cv::Mat& img)
{
    int ret = manifold_cam_read(buffer, &nframe, block);
    if(ret > 0)
    {
        NV12ToRGB(buffer, pData, IMAGE_W, IMAGE_H);
        memcpy(pImg->imageData, pData, FRAME_SIZE);
        img = cv::cvarrToMat(pImg);
        return true;
    }
    else
    {
        return false;
    }
}

void Camera::exit()
{
    while(!manifold_cam_exit())
    {
        sleep(1);
    }
}

sRGB Camera::yuvTorgb(int Y, int U, int V)
{
    sRGB rgb;
    int r = (int)(Y + 1.4075 * (V - 128));
    int g = (int)(Y - 0.3455 * (U - 128) - 0.7169 * (V - 128));
    int b = (int)(Y + 1.779 * (U-128));
    rgb.r = (uint8_t)(r < 0 ? 0: r > 255 ? 255 : r);
    rgb.g = (uint8_t)(g < 0 ? 0: g > 255 ? 255 : g);
    rgb.b = (uint8_t)(b < 0 ? 0: b > 255 ? 255 : b);
    return rgb;
}

void Camera::NV12ToRGB(uint8_t* src, uint8_t* rgb, int width, int height)
{
    int numOfPixel = width * height;
    int positionOfU = numOfPixel;
    int startY, step, startU, Y, U, V, index, nTmp;
    sRGB tmp;

    for(int i = 0;i < height;i++)
    {
        startY = i * width;
        step = i / 2 * width;
        startU = positionOfU + step;
        for(int j = 0;j < width;j++)
        {
            Y = startY + j;
            if(j % 2 == 0)
            {
                nTmp = j;
            }
            else
            {
                nTmp = j - 1;
            }
            U = startU + nTmp;
            V = U + 1;
            index = Y * 3;
            tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
            rgb[index + 0] = tmp.b;
            rgb[index + 1] = tmp.g;
            rgb[index + 2] = tmp.r;
        }
    }
}


};




