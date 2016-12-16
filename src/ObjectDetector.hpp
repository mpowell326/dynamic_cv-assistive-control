
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
    void findSegments();
    void findBlobs();
    
private:
	RsCamera*  camera;
	// cv::SimpleBlobDetector blobDetector;
};


class WatershedSegmenter {
private:
	cv::Mat markers;
	
public:
	void setMarkers(const cv::Mat& markerImage)
	{
		// Convert to image of ints
		markerImage.convertTo(markers,CV_32S);
	}
	
	cv::Mat process(const cv::Mat &image)
	{
		// Apply watershed
		cv::watershed(image,markers);
		return markers;
	
	}
	// Return result in the form of an image
	cv::Mat getSegmentation()
	{
		cv::Mat tmp;
		// all segment with label higher than 255
		// will be assigned value 255
		markers.convertTo(tmp,CV_8U);
		return tmp;
	}
	// Return watershed in the form of an image
	cv::Mat getWatersheds()
	{
		cv::Mat tmp;
		// Each pixel p is transformed into
		// 255p+255 before conversion
		markers.convertTo(tmp,CV_8U,255,255);
		return tmp;
	}
};


#endif  // OBJECT_DETECTOR_H