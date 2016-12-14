

#ifndef ARUCO_MARKER_TRACKER_H
#define ARUCO_MARKER_TRACKER_H


#include <iostream>
#include <string>
#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include "RgbCamera.hpp"
using namespace cv;
using namespace aruco;
using namespace std;

#include <Python.h>
#include <boost/python.hpp>


// -------------------------------------------------------------------------
class arucoTracker : public RgbCamera
{
public:
	arucoTracker();
	arucoTracker(int, float);
	~arucoTracker();

	void detectMarkers();
	void displayMarker();
	void setDetectorParams();
	vector<double> getMarkerPoseXYZ()const{return tracked_marker.Tvec;}
	double getMarkerPoseX();
	double getMarkerPoseY();
	double getMarkerPoseZ();
	int getTrackedMarkerID()const { return tracked_marker_id;}
	void calibrateCameraFromFile(char*);

private:
	MarkerDetector MDetector;
	vector<Marker> Markers;
	MarkerPoseTracker MTracker;
	int tracked_marker_id;
	float marker_size;
	Marker tracked_marker;
    CameraParameters camParam;

	Marker estimateMarkerPose(Marker);
};


// -------------------------------------------------------------------------
// Boost python wrapper to allow libary to be imported to python script
// -------------------------------------------------------------------------
BOOST_PYTHON_MODULE(lib_arucoTracker)
{    
	using namespace boost::python;

    class_<arucoTracker>("arucoTracker",init<>())
    	.def(init<int, float>())
        .def("detectMarkers", &arucoTracker::detectMarkers)
     	.def("getMarkerPoseXYZ", &arucoTracker::getMarkerPoseXYZ)  
     	.def("displayMarker", &arucoTracker::displayMarker)  
     	.def("setDetectorParams", &arucoTracker::setDetectorParams)
     	.def("getTrackedMarkerID", &arucoTracker::getTrackedMarkerID)
     	.def("openCaptureDevice", &arucoTracker::openCaptureDevice)
     	.def("calibrateCameraFromFile", &arucoTracker::calibrateCameraFromFile)
        .def("getNextFrame", &arucoTracker::getNextFrame)
        .def("getMarkerPoseX", &arucoTracker::getMarkerPoseX)
        .def("getMarkerPoseY", &arucoTracker::getMarkerPoseY)
        .def("getMarkerPoseZ", &arucoTracker::getMarkerPoseZ)
    ;
}



#endif  // ARUCO_MARKER_TRACKER_H