#include "RgbCamera.hpp"


void RgbCamera::openCaptureDevice(int deviceNum)
{
	videoCaptureDevice.open(deviceNum);

	// grab a first frame to check if everything is ok
	videoCaptureDevice>>frame;

	if ( frame.empty() )
	{
	    std::cout << "Input media could not be loaded, aborting\n";
	    std::exit(-1);
	}
}

void RgbCamera::getNextFrame()
{
	videoCaptureDevice >> frame;
	if( frame.empty() )
    {
        std::cout << "Can't read frames from the camera\n";
    }
}

void RgbCamera::displayFrame(void)
{
	cv::namedWindow( "Video", 1 );
	cv::imshow( "Video", frame );
}
