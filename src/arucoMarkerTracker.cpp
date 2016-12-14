#include "arucoMarkerTracker.hpp"



#define TRACKED_MARKER_ID 	244


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


void arucoTracker::calibrateCameraFromFile(char* filepath)
{
	camParam.readFromXMLFile(filepath);
	camParam.resize(frame.size());
}

Marker arucoTracker::estimateMarkerPose(Marker marker)
{
	MTracker.estimatePose( marker, camParam, marker_size );//call its tracker and estimate the pose
	return marker;
}

void arucoTracker::detectMarkers()
{
	if (camParam.isValid() && marker_size != -1)
	{
		MDetector.detect( frame, Markers, camParam, marker_size, 0.1 );
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
    if (camParam.isValid() && marker_size != -1)
    {
        CvDrawingUtils::draw3dAxis(frame, tracked_marker, camParam);
        CvDrawingUtils::draw3dCube(frame, tracked_marker, camParam);
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

// int main(int argc, char const *argv[])
// {
// 	arucoTracker ArucoTracker(TRACKED_MARKER_ID, 0.032);

// 	ArucoTracker.openCaptureDevice(0);
// 	ArucoTracker.setDetectorParams();
// 	ArucoTracker.calibrateCameraFromFile((char*)"../aruco_testproject/cameraCalibration.yml");

// 	while(true)
// 	{
// 		ArucoTracker.getNextFrame();
// 		try
// 		{
// 			ArucoTracker.detectMarkers();
// 			ArucoTracker.displayMarker();
// 		}
// 		catch (std::exception &ex)
// 		{
// 			cout<<"Exception :"<<ex.what()<<endl;
// 		}

// 		if (ArucoTracker.getTrackedMarkerID() == TRACKED_MARKER_ID)
// 		{
// 			cout << ArucoTracker.getMarkerPoseXYZ()[0] << endl;
			
// 		}

// 		if(cv::waitKey(1000.0/ArucoTracker.fps) == 27) break;
// 	}
// 	return 0;
// }
