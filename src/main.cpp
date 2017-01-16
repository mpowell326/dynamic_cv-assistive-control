
#include "RsCamera.hpp"
#include "ObjectDetector.hpp"


#define CAMERA_HEIGHT   0.6 // Camera mounting offset from floor (m)
#define CAMERA_ANGLE    20.0  // Camera mounting angle (deg). Positive is rotated torwards ground.
#define VOXEL_LEAFSIZE  0.02f
#define FLOOR_ANGLE_EPS 30.0


/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main() try
{
    RsCamera rsCamera;
    ObjectDetector objectDetector(&rsCamera);
    Displayer display(&rsCamera);
    PCLViewer pclViewer;

    std::vector<std::vector<std::pair<double, double>>> osbtacle_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr              xyzrgb_cloud(  new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 filteredCloud(  new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 floor_area_cloud(   new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 obstacles_cloud(    new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 flattened_cloud(  new pcl::PointCloud<pcl::PointXYZ>);


    rs::log_to_console( rs::log_severity::warn );


    if( !rsCamera.startStreaming( ) )
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console( rs::log_severity::fatal );
        return EXIT_FAILURE;
    }




    display.initializeWindows();
    // pclViewer.addRGBCloud<pcl::PointXYZRGB>( xyzrgb_cloud,  "cloud",        1 );
    pclViewer.addXYZCloud<pcl::PointXYZ>( filteredCloud,    "orig",         1,  1,1,1 );
    pclViewer.addXYZCloud<pcl::PointXYZ>( floor_area_cloud, "floor",        4,  0,0,1 );
    pclViewer.addXYZCloud<pcl::PointXYZ>( obstacles_cloud,  "obstacles",    4,  1,0,0 );

    while( true )
    {
        /* ==== Get next frames (color, depth, point cloud, etc. ) ==== */
        rsCamera.getNextFrame();


        // cout<<"Close? "<<objectDetector.isObjectClose()<<endl;



        /* ==== Point Cloud Processing, etc. ==== */

        pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(* rsCamera.getPointCloud(), *xyzrgb_cloud ) ;
        // xyzrgb_cloud = rsCamera.getPointCloud();

        /* Downsample to increase fps */
        objectDetector.applyVoxelGrid(xyzrgb_cloud, VOXEL_LEAFSIZE);
        objectDetector.convertRGBtoXYZ(xyzrgb_cloud, filteredCloud);

        /* Transform input cloud to account for camera mounting */
        objectDetector.transform(filteredCloud,  filteredCloud);

        /* Filter point cloud to remove noise and downsample */
        objectDetector.applyPassThroughZ(filteredCloud, MAX_RANGE);
        objectDetector.removeOutliers(filteredCloud);

        // /* Detect the floor plane */
        objectDetector.findFloor(filteredCloud, floor_area_cloud, obstacles_cloud);
        objectDetector.flattenCloud(obstacles_cloud, flattened_cloud);
        objectDetector.cluster2Dcloud(flattened_cloud, osbtacle_map );



        /* ==== Display streams, clouds, plots ==== */
        display.displayFps(rsCamera.getFps());
        pclViewer.updatePlot(osbtacle_map);
        pclViewer.display();
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
