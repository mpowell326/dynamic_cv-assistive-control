
#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
using namespace std;

// -------------------------------------------------------------------------

class Camera
{
public:
	
	void getNextFrame();

	// cv::Mat				frame;
	double			    fps = 60;
	int				    frameWidth = 320;
	int				    frameHeight = 240;
};




#endif  // CAMERA_H