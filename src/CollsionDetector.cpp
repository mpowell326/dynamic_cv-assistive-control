#include "CollsionDetector.hpp"


CollisionDetector::CollisionDetector(){}

void CollisionDetector::start()
{
    objectDetector.setCamera(&rsCamera);

    rs::log_to_console( rs::log_severity::warn );

    cout<<"starting"<<endl;
    if( !rsCamera.initializeStreaming() )
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console( rs::log_severity::fatal );
        // return EXIT_FAILURE;
    }

    rsCamera.setupWindows();
}


void CollisionDetector::update()
{
	try
	{
	    // if( rsCamera._device.is_streaming( ) )

	    rsCamera.getNextFrame();
	    objectDetector.isObjectClose();
	    rsCamera.displayStreams();
	    
	    // rsCamera.displayStreamsGL();
	    rsCamera.displayFPS();

    }
    catch( const rs::error & e )
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        // return EXIT_FAILURE;
    }
    catch( const std::exception & e )
    {
        std::cerr << e.what() << std::endl;
        // return EXIT_FAILURE;
    }
}

CollisionDetector::~CollisionDetector()
{
    rsCamera.disableStreaming();
    cv::destroyAllWindows();
}

// void CollisionDetector::stop()
// {
//     rsCamera.disableStreaming();
//     cv::destroyAllWindows( );
//     // return EXIT_SUCCESS;
// }


