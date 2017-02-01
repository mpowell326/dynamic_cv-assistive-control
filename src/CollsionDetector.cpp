#include "CollsionDetector.hpp"


// rs::context rsContex; 

// CollisionDetector::CollisionDetector(){


//     // rsCamera = RsCamera();
//     // objectDetector = ObjectDetector();
// }

void CollisionDetector::start()
{
    rs::log_to_console( rs::log_severity::warn );

    if( !rsCamera.startStreaming( ) )
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console( rs::log_severity::fatal );
        // return EXIT_FAILURE;
    }

    // display.initializeWindows();
}


void CollisionDetector::update()
{
	try
	{
        /* ==== Get next frames (color, depth, point cloud, etc. ) ==== */
        rsCamera.getNextFrame();



        /* ==== Point Cloud Processing, etc. ==== */

        pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(* rsCamera.getPointCloud(), *xyzrgb_cloud ) ;

        /* Downsample to increase fps */
        objectDetector.applyVoxelGrid(xyzrgb_cloud, VOXEL_LEAFSIZE);
        objectDetector.convertRGBtoXYZ(xyzrgb_cloud, filteredCloud);

        /* Transform input cloud to account for camera mounting */
        objectDetector.transform(filteredCloud,  filteredCloud);

        /* Filter point cloud to remove noise */
        objectDetector.applyPassThroughZ(filteredCloud, MAX_RANGE);
        objectDetector.removeOutliers(filteredCloud);

        /* Detect the floor plane */
        objectDetector.findFloor(filteredCloud, floor_area_cloud, obstacles_cloud);
        objectDetector.flattenCloud(obstacles_cloud, flattened_cloud);
        // objectDetector.cluster2Dcloud(flattened_cloud, obstacle_map );
        objectDetector.generateObstacleMap_grid<int>( obstacles_cloud, obstacle_map, mapWidth, mapDepth );

        std::vector<std::pair<double, double>> obstacle_plot;
        obstacle_plot.clear();
        for(int i = 0; i < mapWidth; i++)
        {
            for( int j=0; j<mapDepth; j++)
            {
                if( obstacle_map[i][j] == 1)
                {
                    obstacle_plot.push_back(std::pair<double, double> (i, j));
                }
            }
        }
        // pclViewer.updatePlot(obstacle_plot);

        /* ==== Display streams, clouds, plots ==== */
        display.displayFps(rsCamera.getFps());
        // pclViewer.updatePlot(obstacle_map);
        // pclViewer.display();
        // display.displayStreams();

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
    rsCamera.stopStreaming();
    cv::destroyAllWindows();
}

void CollisionDetector::stop()
{
    rsCamera.stopStreaming();
    cv::destroyAllWindows( );
    // return EXIT_SUCCESS;
}


