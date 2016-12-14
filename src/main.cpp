
#include "RsCamera.hpp"
#include "ObjectDetector.hpp"


/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main( ) try
{
    // rs::context rsContex; 
    // RsCamera  rsCamera(&rsContex);
    RsCamera rsCamera;
    ObjectDetector objectDetector(&rsCamera);

    rs::log_to_console( rs::log_severity::warn );


    if( !rsCamera.initializeStreaming( ) )
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console( rs::log_severity::fatal );
        return EXIT_FAILURE;
    }

    rsCamera.setupWindows();
    // rsCamera.setupWindowsGL();

    while( true )
    {
        // if( rsCamera._device->is_streaming( ) )

        rsCamera.getNextFrame();
        objectDetector.isObjectClose();
        rsCamera.displayStreams();
        
        // rsCamera.displayStreamsGL();
        rsCamera.displayFPS();
    }

    rsCamera.disableStreaming();
    cv::destroyAllWindows( );
    return EXIT_SUCCESS;





}
catch( const rs::error & e )
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch( const std::exception & e )
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
