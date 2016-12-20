
#include "RsCamera.hpp"
#include "ObjectDetector.hpp"


/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main( ) try
{
    RsCamera rsCamera;
    ObjectDetector objectDetector(&rsCamera);
    Displayer display(&rsCamera);

    rs::log_to_console( rs::log_severity::warn );


    if( !rsCamera.startStreaming( ) )
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console( rs::log_severity::fatal );
        return EXIT_FAILURE;
    }

    display.initializeWindows();

    while( true )
    {
        rsCamera.getNextFrame();
        cout<<"Close? "<<objectDetector.isObjectClose()<<endl;
        // display.displayFps(rsCamera.getFps());
        display.displayStreams();

    }

    rsCamera.stopStreaming();
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
