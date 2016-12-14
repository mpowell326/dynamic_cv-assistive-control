#ifndef RS_CAMERA_H
#define RS_CAMERA_H


#include "Camera.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense/rs.hpp>
#include "TextureBuffer.hpp"
using namespace std;
// using namespace rs;

// Also include GLFW to allow for graphical display
#include <GLFW/glfw3.h>


#define STREAMS 4


class RsCamera : public Camera
{
public:
    RsCamera();
    // RsCamera(rs::context* context);
	bool initializeStreaming();
    bool disableStreaming();
    void startStream(rs_stream stream, rs_format format);
    void uploadFrames();
    void getNextFrame();
    void setupWindows();
    void setupWindowsGL();
    void displayStreams();
    void displayStreamsGL();
    void ConvertRsframe2OpenCVMat(rs::stream stream, const void * data, cv::Mat *outImg);
    cv::Mat* getDepthFrame() { return &rawStreamData[(int)rs::stream::depth] ; }
    void displayFPS() { cout << currentFps << endl; }
    void convertDepthMat4Display(cv::Mat* input, cv::Mat* output);

private:
	rs::device* 	_device = NULL;
    rs::context _rsCtx;             // Create a context object. This object owns the handles to all connected realsense devices.
    static const int streams = 4;
    
    std::vector<uint16_t> supported_streams = { (uint16_t)rs::stream::depth, (uint16_t)rs::stream::color, (uint16_t)rs::stream::infrared};
    texture_buffer buffers[streams];
    GLFWwindow * glWin;
    struct resolution
    {
        int width;
        int height;
        rs::format format;
    };
    std::map<rs::stream, resolution> resolutions;
    

    cv::Mat rawStreamData[streams];
    cv::Mat output;

    /* FPS tracking */
    double last_timestamp;

    int currentFps=0,num_frames=0;
    double next_time;
};






#endif  // RS_CAMERA_H
