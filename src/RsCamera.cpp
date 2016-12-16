#include "RsCamera.hpp"



// Named windows
#define WINDOW_DEPTH  "Depth Image"
#define WINDOW_RGB	  "RGB Image"
#define WINDOW_IR	  "IR Image"

RsCamera::RsCamera(){}
// RsCamera::RsCamera(rs::context* context)
// {
//     _rsCtx = context;
// }

bool RsCamera::initializeStreaming()
{
    bool success = false;

	printf("There are %d connected RealSense devices.\n", _rsCtx.get_device_count());
    if(_rsCtx.get_device_count() != 0)
    {
        _device = _rsCtx.get_device(0);

        printf("\nUsing device 0, an %s\n", _device->get_name());
        printf("    Serial number: %s\n", _device->get_serial());
        printf("    Firmware version: %s\n", _device->get_firmware_version());

        /* Configure all streams to run at VGA resolution at 60 frames per second */
        _device->enable_stream(rs::stream::depth, 0, 0, rs::format::disparity16, fps, rs::output_buffer_format::native);
        _device->enable_stream(rs::stream::color, frameWidth, frameHeight, rs::format::rgb8, fps, rs::output_buffer_format::native);
        _device->enable_stream(rs::stream::infrared, 0, 0, rs::format::y8, fps, rs::output_buffer_format::native);
        if (_device->supports(rs::capabilities::infrared2))
            _device->enable_stream(rs::stream::infrared2, 0, 0, rs::format::y8, fps, rs::output_buffer_format::native);

        resolutions[rs::stream::depth] = { _device->get_stream_width(rs::stream::depth), _device->get_stream_height(rs::stream::depth), rs::format::z16 };
        resolutions[rs::stream::color] = { _device->get_stream_width(rs::stream::color), _device->get_stream_height(rs::stream::color), rs::format::rgb8 };
        resolutions[rs::stream::infrared] = { _device->get_stream_width(rs::stream::infrared), _device->get_stream_height(rs::stream::infrared), rs::format::y8 };
        if(_device->supports(rs::capabilities::infrared2))
            supported_streams.push_back((uint16_t)rs::stream::infrared2);
            resolutions[rs::stream::infrared2] = { _device->get_stream_width(rs::stream::infrared2), _device->get_stream_height(rs::stream::infrared2), rs::format::y8 };

        // cout<<"Depth Clamp"<<endl;
        // cout<< resolutions[rs::stream::depth].width <<endl;
        // cout<< _device->get_option(rs::option::r200_depth_clamp_min) <<endl;

        cout<< "Motion?: " << _device->supports(rs::capabilities::motion_events) <<endl;
        rs::apply_depth_control_preset(_device, 4);
        _device->start();

        success = true;
    }
    return success;
}


bool RsCamera::disableStreaming()
{
    _device->stop();

    for (auto i : supported_streams)
    {
        if (_device->is_stream_enabled((rs::stream)i))
            _device->disable_stream((rs::stream)i);
    }

    return true;
}

 
void RsCamera::ConvertRsframe2OpenCVMat(rs::stream stream, const void * data, cv::Mat *outImg)
{
    int cvDataType;
    int cvDataWidth;

    switch (_device->get_stream_format( stream ))
    {
        case rs::format::any:
            throw std::runtime_error("not a valid format");
            break;
        case rs::format::xyz32f:            //< 32 bit floating point 3D coordinates.
            cvDataType = CV_32F;
            cvDataWidth = 4;
            break;

        /* STREAM_TYPE_DEPTH */
        case rs::format::z16:               //< 16 bit linear depth values. The depth is meters is equal to depth scale * pixel value
        case rs::format::disparity16:       //< 16 bit linear disparity values. The depth in meters is equal to depth scale / pixel value
            cvDataType = CV_16U;
            cvDataWidth = 2;
            break;

        /* STREAM_TYPE_COLOR */
        case rs::format::rgb8:
        case rs::format::bgr8:
            cvDataType = CV_8UC3;
            cvDataWidth = 3;
            break;
        case rs::format::rgba8:
        case rs::format::bgra8:   
            cvDataType = CV_8UC4;
            cvDataWidth = 4;
            break;
        case rs::format::yuyv:
        case rs::format::raw10:             //< Four 10-bit luminance values encoded into a 5-byte macropixel
        case rs::format::raw16:             //< Four 10-bit luminance filled in 16 bit pixel (6 bit unused)
            throw(0); // Not implemented
            break;
        
        /* STREAM_TYPE_IR */
        case rs::format::y8:                /* Relative IR Image */
            cvDataType = CV_8U;
            cvDataWidth = 1;
            break;
        case rs::format::y16:               /* 16-Bit Gray Image */
            cvDataType = CV_16U;
            cvDataWidth = 2;
            break;

        /* Monochrome Wide FOW */
        case rs::format::raw8:
            throw(0); // Not implemented
            break;

        default:
            throw std::runtime_error("The requested format is not provided by demo");
            break;
    }

    int h = _device->get_stream_height(stream);
    int w = _device->get_stream_width(stream);
    outImg->create( h, w, cvDataType);
    memcpy(outImg->data, (uchar*)reinterpret_cast<const uint8_t *>(data), h*w*cvDataWidth);

    if(stream == rs::stream::depth)
    {
        outImg->convertTo( (* outImg), CV_8UC1, 1.0f/ _device->get_depth_scale() );
    }
    if( stream == rs::stream::color)
    {
        cv::cvtColor( (* outImg), (* outImg), cv::COLOR_BGR2RGB );
    }
}


void RsCamera::getNextFrame()
{
    const void * rawFrame;
    std::vector<uint8_t> rgb;
    const double timestamp = _device->get_frame_timestamp(rs::stream::color);

    _device->wait_for_frames();

    if(timestamp != last_timestamp)
    {
        for (auto i : supported_streams)
        {
         //   auto res = resolutions[(rs::stream)i];
            rawFrame = _device->get_frame_data( (rs::stream)i );
            ConvertRsframe2OpenCVMat( (rs::stream)i, rawFrame , &rawStreamData[i] );

            /* Upload to Gl buffers for displaying */
            buffers[i].upload(*_device, (rs::stream)i);
        }

        last_timestamp = timestamp;
        ++num_frames;
        if(timestamp >= next_time)
        {
            currentFps = num_frames;
            num_frames = 0;
            next_time += 1000;
        }
    }
}


void RsCamera::uploadFrames()
{
    for (auto i : supported_streams)
    {
        buffers[i].upload(*_device, (rs::stream)i);
    }
}


void  RsCamera::setupWindows()
{
    cv::namedWindow( WINDOW_DEPTH, 0 );
    cv::namedWindow( "Thresholded", 0 );
}


void RsCamera::setupWindowsGL()
{
    // Open a GLFW window to display our output
    glfwInit();

//     auto max_aspect_ratio = 0.0f;
//     for (auto i : supported_streams)
//     {
//         auto aspect_ratio = static_cast<float>(resolutions[static_cast<rs::stream>(i)].height) / static_cast<float>(resolutions[static_cast<rs::stream>(i)].width);
//         if (max_aspect_ratio < aspect_ratio)
//             max_aspect_ratio = aspect_ratio;
//     };
//     glWin = glfwCreateWindow(1100, int(1100 * max_aspect_ratio), "librealsense - stride", nullptr, nullptr);

    glWin = glfwCreateWindow(frameWidth*2, frameHeight*2, "Realsense camera output", nullptr, nullptr);
    glfwMakeContextCurrent(glWin);
}


void RsCamera::displayStreamsGL()
{

//         glfwPollEvents();

    int w, h;
    glfwGetFramebufferSize(glWin, &w, &h);
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT);

    glfwGetWindowSize(glWin, &w, &h);
    glLoadIdentity();
    glOrtho(0, w, h, 0, -1, +1);

    for (auto i = 0; i < streams; i++)
    {
        // if(!_device->supports(rs::capabilities(i))) continue;

        auto res = resolutions[(rs::stream)i];

        auto x = (i % 2) * (w / 2);
        auto y = (i / 2) * (h / 2);
        buffers[i].show(x, y, w / 2, h / 2, res.width, res.height);
    }

    // glPixelZoom(1, -1);

    // // Display depth data by linearly mapping depth between 0 and 2 meters to the red channel
    // glRasterPos2f(-1, 1);
    // glPixelTransferf(GL_RED_SCALE, 0xFFFF * _device->get_depth_scale() / 2.0f);
    // glDrawPixels(frameWidth, frameHeight, GL_RED, GL_UNSIGNED_SHORT, _device->get_frame_data(rs::stream::depth));
    // glPixelTransferf(GL_RED_SCALE, 1.0f);

    // // Display color image as RGB triples
    // glRasterPos2f(0, 1);
    // glDrawPixels(frameWidth, frameHeight, GL_RGB, GL_UNSIGNED_BYTE, _device->get_frame_data(rs::stream::color));

    // // Display infrared image by mapping IR intensity to visible luminance
    // glRasterPos2f(-1, 0);
    // glDrawPixels(frameWidth, frameHeight, GL_LUMINANCE, GL_UNSIGNED_BYTE, _device->get_frame_data(rs::stream::infrared));

    glfwSwapBuffers(glWin);
}

void RsCamera::convertDepthMat4Display(cv::Mat* input, cv::Mat* output)
{
    input->convertTo( (* output), CV_8UC1, 100.0f);
    // applyColorMap((* output), (* output), cv::COLORMAP_WINTER);
}
void RsCamera::displayStreams()
{
    cv::Mat tmp;

    convertDepthMat4Display( &rawStreamData[(int)rs::stream::depth], &tmp );
    imshow( WINDOW_DEPTH, tmp );
    cvWaitKey( 1 );

    // 
    // cv::imshow( WINDOW_RGB, rawStreamData[(int)rs::stream::color] );
    // cvWaitKey( 1 );

}

