#include <iostream>
#include <string>
#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace aruco;
using namespace std;


// #define TRACKED_MARKER_ID 	244


// -------------------------------------------------------------------------

class Camera
{
public:
	VideoCapture	videoCaptureDevice;
	Mat				frame;
	double			fps;
	int				frame_width;
	int				frame_height;
	CameraParameters CamParam;

	Camera(void);
	void getNextFrame();	
	void openCaptureDevice(int);
	void calibrateFromFile(char*);
	void displayFrame(void);
};

// -------------------------------------------------------------------------

class arucoTracker : public Camera
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

private:
	MarkerDetector MDetector;
	vector<Marker> Markers;
	MarkerPoseTracker MTracker;
	int tracked_marker_id;
	float marker_size;
	Marker tracked_marker;

	Marker estimateMarkerPose(Marker);
};
