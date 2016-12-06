#include <iostream>
#include <string>
#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include "arucoMarkerTracker.hpp"
using namespace cv;
using namespace aruco;
using namespace std;


#define TRACKED_MARKER_ID 	244


// -------------------------------------------------------------------------
Camera::Camera(void)
{
	fps = 24.0;
}

void Camera::openCaptureDevice(int deviceNum)
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

void Camera::getNextFrame()
{
	videoCaptureDevice >> frame;
	if( frame.empty() )
    {
        std::cout << "Can't read frames from the camera\n";
    }
}

void Camera::displayFrame(void)
{
	namedWindow( "Video", 1 );
	imshow( "Video", frame );
}

void Camera::calibrateFromFile(char* filepath)
{
	CamParam.readFromXMLFile(filepath);
	CamParam.resize(frame.size());
}

// -------------------------------------------------------------------------

arucoTracker::arucoTracker()
{
	tracked_marker_id = 244;
	marker_size = -1;
}

arucoTracker::arucoTracker(int id, float size)
{
	tracked_marker_id = id;
	marker_size = size;
}

arucoTracker::~arucoTracker(){}

void arucoTracker::setDetectorParams()
{
	MDetector.setDictionary("ARUCO_MIP_36h12");
    MDetector.setThresholdParams(20, 7);
    MDetector.setThresholdParamRange(2, 0);
}

Marker arucoTracker::estimateMarkerPose(Marker marker)
{
	MTracker.estimatePose( marker, CamParam, marker_size );//call its tracker and estimate the pose
	return marker;
}

void arucoTracker::detectMarkers()
{
	if (CamParam.isValid() && marker_size != -1)
	{
		MDetector.detect( frame, Markers, CamParam, marker_size, 0.1 );
	}
	else
	{
		 Markers = MDetector.detect( frame );
	}
	tracked_marker = NULL;
	for( auto & marker:Markers )//for each marker
	{
	    if( marker.id == tracked_marker_id )
	    {
	    	tracked_marker = estimateMarkerPose(marker);
	    }
	}
}

void arucoTracker::displayMarker()
{       
    tracked_marker.draw(frame, Scalar(0, 0, 255), 2);
    
    // draw a 3d cube if there is 3d info
    if (CamParam.isValid() && marker_size != -1)
    {
        CvDrawingUtils::draw3dAxis(frame, tracked_marker, CamParam);
        CvDrawingUtils::draw3dCube(frame, tracked_marker, CamParam);
    }
	
	namedWindow( "Marker", 1 );
	imshow( "Marker", frame );
}

double arucoTracker::getMarkerPoseX()
{
	std::vector<double> vecXYZ;
	vecXYZ = tracked_marker.Tvec;

	return vecXYZ[0];
}

double arucoTracker::getMarkerPoseY()
{
	std::vector<double> vecXYZ;
	vecXYZ = tracked_marker.Tvec;

	return vecXYZ[1];
}
double arucoTracker::getMarkerPoseZ()
{
	std::vector<double> vecXYZ;
	vecXYZ = tracked_marker.Tvec;

	return vecXYZ[2];
}
// -------------------------------------------------------------------------

int main(int argc, char const *argv[])
{
	arucoTracker ArucoTracker(TRACKED_MARKER_ID, 0.032);

	ArucoTracker.openCaptureDevice(0);
	ArucoTracker.setDetectorParams();
	ArucoTracker.calibrateFromFile((char*)"../aruco_testproject/cameraCalibration.yml");

	while(true)
	{
		ArucoTracker.getNextFrame();
		try
		{
			ArucoTracker.detectMarkers();
			ArucoTracker.displayMarker();
		}
		catch (std::exception &ex)
		{
			cout<<"Exception :"<<ex.what()<<endl;
		}

		if (ArucoTracker.getTrackedMarkerID() == TRACKED_MARKER_ID)
		{
			cout << ArucoTracker.getMarkerPoseXYZ()[0] << endl;
			
		}

		if(cv::waitKey(1000.0/ArucoTracker.fps) == 27) break;
	}
	return 0;
}

// -------------------------------------------------------------------------

// class CameraAssist
// {
// public:
// 	std::vector<int> jsDemands;
// 	std::vector<int> adjDemands;
// 	Camera RGBcamera;
// 	Camera RScamera;
// 	arucoTracker Aruco_tracker;
// 	int mode;

// 	CameraAssist();
// 	~CameraAssist();
// 	void updateJSdemands(int x, int y);
// 	void openCaptureDevice(char* filepath);
// 	void getAdjdemands(void);
// 	void calculateAdjdemands(void);

// };

// // void CameraAssist::openCaptureDevice(char* filepath)
// // {
// // 	videoCapture.open(filepath);

// // 	// grab a first frame to check if everything is ok
// // 	videoCapture>>frame;

// // 	if ( assert(frame.empty()) )
// // 	{
// // 	    std::cout << "Input media could not be loaded, aborting\n";
// // 	    std::exit(-1);
// // 	}
// // }

// void CameraAssist::updateJSdemands(int x, int y)
// {
// 	jsDemands[0] = x;
// 	jsDemands[1] = y;
// }

// void CameraAssist::calculateAdjdemands(void)
// {
// 	// switch(mode)
// 	// {
// 	// 	case AVOID_COLLISION:
// 	// 	{
// 			Aruco_tracker.

// 			if (x_restrict != None and y_restrict != None)
// 			{
// 			    if( jsDemands[1] > 0 and y_restrict >0)
// 			    {
// 			        adjDemands[1] = self.jsYdemand * (100-y_restrict)/100;
// 			    }
// 			    else
// 			    {
// 			        self.adjYdemand = self.jsYdemand;
// 			    }

// 			    if( self.jsXdemand < 0 and x_restrict < 0 )
// 			    {
// 			        self.adjXdemand = self.jsXdemand * (100-abs(x_restrict))/100;
// 			    }
// 			    else( self.jsXdemand > 0 and x_restrict > 0)
// 			    {
// 			        self.adjXdemand = self.jsXdemand * (100-abs(x_restrict))/100;    
// 			    }
// 			    else
// 			    {
// 			        self.adjXdemand = self.jsXdemand;
// 			    }
// 		    }
// 	// 	}
// 	// }			
// }
