
#ifndef RGB_CAMERA_H
#define RGB_CAMERA_H

#include "Camera.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


class RgbCamera : public Camera
{
public:
    cv::VideoCapture    videoCaptureDevice;
       
    void openCaptureDevice(int);
    void displayFrame(void);
    void getNextFrame();
};



#endif  // RGB_CAMERA_H