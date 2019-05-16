#ifndef __DETECTOR_H
#define __DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>
#include <cmath>
#include "coordinatetransform.h"

#define DICT_4X4_50 0
#define DICT_5X5_50 4
#define DICT_6X6_50 8

using namespace std;
using namespace cv;
using namespace aruco;

namespace detector{

class Detector
{
public:
    // Interface function
    bool init(string Config_File, string Param_File);
    void detect(Mat& img, vector<int>& ids);
    void drawmarker(Mat& img, CvScalar color = Scalar(0, 0, 255), int thickness = 10);
    void locate(double yaw, double gimbalpitch, ct::GlobalPosition& globalpos0, vector<ct::GlobalPosition>& globalpositions);
private:
    //

private:
    // Object
    Ptr<DetectorParameters> detectorParams;
    Ptr<Dictionary> dictionary;
    vector<vector<Point2f> > corners;
    Mat cameraMatrix; // 内参矩阵
    Mat distCoeffs; // 畸变系数k1,k2,p1,p2,k3
    Mat Rcam2uav; // 相机坐标到无人机坐标的旋转矩阵
    Mat Tcam2uav; // 相机坐标到无人机坐标的平移向量

    // Parameter
    double markerLength;
};

template<typename T>
void matread(ifstream& fin, Mat& mat);
template<typename T>
void matwrite(ofstream& fout, Mat& mat);

void readCameraParameters(string filename, Mat& cameraMatrix, Mat& distCoeffs);
bool readDetectorParameters(string filename, Ptr<DetectorParameters>& params);


}

#endif // __DETECTOR_H
