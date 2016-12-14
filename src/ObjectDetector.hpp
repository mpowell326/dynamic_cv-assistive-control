
#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "RsCamera.hpp"
using namespace std;
using namespace cv;

class ObjectDetector : public RsCamera
{
public:
	ObjectDetector();
    ObjectDetector(RsCamera * ptr);

    void setCamera(RsCamera* ptr);
    
    bool isObjectClose();

    
private:
	RsCamera*  camera;
	// cv::SimpleBlobDetector blobDetector;
};





#endif  // OBJECT_DETECTOR_H