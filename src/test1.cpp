/* ------------------------------------------------------------------ *
 	Computer Vision testing file

 	Date: 	Nov 2016
 	Author: Morgan
 * ------------------------------------------------------------------ */
 	
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	cv::VideoCapture cap(0);
	if(!cap.isOpened())
	{
		std::cout << "Unable to open the camera\n";
		std::exit(-1);
	}

	cv::Mat raw_vid;
	cv::Mat proccesed_vid;
	double FPS = 24.0;

	while(true)
	{
		cap >> raw_vid;
		if(raw_vid.empty())
		{
			std::cout << "Can't read frames from the camera\n";
			break;
		}


		cv::Mat kernel = getGaussianKernel(50, 1);
		kernel = getStructuringElement(MORPH_RECT, Size(25,25));
		// dilate(raw_vid, proccesed_vid,kernel);
		erode(raw_vid, proccesed_vid,kernel);
		// blur(proccesed_vid, proccesed_vid, cv::Size(30, 30) );
		cv::imshow("Camera feed", proccesed_vid);

		if(cv::waitKey(1000.0/FPS) == 27) break;
	}

	return 0;
}
