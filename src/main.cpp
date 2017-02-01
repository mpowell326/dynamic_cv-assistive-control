
#include "RsCamera.hpp"
#include "ObjectDetector.hpp"
#include "common.hpp"
#include <boost/python.hpp>
#include <Python.h>
#include <stack>
// #include "camera_tool.v4.py"




// template<typename T>
// struct custom_tuple_to_pair{
//     static PyObject* convert(const boost::python::tuple t){
//         std::pair<T,T> ret;

//         ret.first = t[0];
//         ret.second = t[1];

//         return boost::python::incref(ret.ptr());
//     }
// };

std::pair<double, double> PointOnCircle(double radius, double angleInRadians, double x_origin, double y_origin)
{
    // Convert from degrees to radians via multiplication by PI/180        
    double x = (double)(radius * cos(angleInRadians )) + x_origin;
    double y = (double)(radius * sin(angleInRadians )) + y_origin;
    return std::pair<double, double> (x, y);
}

void generateArc(double radius, double angleInRadians, double y, std::vector<std::pair<double,double>>& arc)
{
    double d_angle;
    int i;
    double y_origin = 0;
    double x_origin = 200;

    if (y>0){
        if (angleInRadians == 0)
        {
            for( i=1; i <= 50; i++ )
            {
                arc.push_back(std::pair<double, double> (x_origin, y*i/50) );
            }
        }
        else
        {
        x_origin += radius;
            if (angleInRadians > 0 )
            {
                for( i=1; i <= 50; i++ )
                {
                    d_angle = (M_PI) - (angleInRadians *i/50);
                    arc.push_back(PointOnCircle(radius, d_angle ,x_origin, y_origin));
                }
            }
            else if (angleInRadians < 0 )
            {
                for( i=1; i <= 50; i++ )
                {
                    d_angle = (-angleInRadians *i/50);
                    arc.push_back(PointOnCircle(abs(radius), d_angle ,x_origin ,y_origin));
                }
            }
        }
    }
}

// boost::python::from_python_converter<   boost::python::tuple, custom_tuple_to_pair<int>     >();
// custom_tuple_to_pair<int>  ();
/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main() try
{
    int xDemand =0, yDemand = 0, adjX=0, adjY=0;
    double vel_linear=0, vel_turn=0,  theta=0;

    double R=0,x=0,y=0,dt=1,d=0;

    setenv("PYTHONPATH", "/home/morgan/src/unidrive-edit/lib/platform/tools/can/", 1);
    // setenv("PYTHONPATH", "/home/morgan/src/unidrive-edit/lib/platform/tools/", 0);



    /* Initalize python-can interface */
    Py_Initialize();
    // boost::python::import("sys").attr("path").attr("append")("/home/morgan/src/unidrive-edit/lib/platform/tools/can/");
    // boost::python::import("sys").attr("path").attr("append")("/home/morgan/src/unidrive-edit/lib/platform/tools/can/../../pyIntegrationTestFramework/");

    // boost::python::object main_module(boost::python::handle<>(boost::python::borrowed(PyImport_AddModule("__main__"))));

    boost::python::object main_module, main_namespace, canCameraInteraface_module, canCameraInteraface,temp; 
    

    // main_module(boost::python::handle<>(boost::python::borrowed(PyImport_AddModule("__main__"))));
    main_module = boost::python::import("__main__");
    assert(main_module);
    main_namespace = main_module.attr("__dict__");
    assert(main_namespace);

    exec(
           "import camera_can_interface\n"
           "canCameraInteraface = camera_can_interface.CanCameraInteraface()\n"
         , main_namespace
       );
    // canCameraInteraface_module = boost::python::import("camera_can_interface");

    // canCameraInteraface = canCameraInteraface_module.attr("CanCameraInteraface")();
    canCameraInteraface = main_namespace["canCameraInteraface"];





    // try{
    //     temp = canCameraInteraface.attr("getJsXdemand")() ;
    //     xDemand = boost::python::extract<int>(  temp  )();
    //     temp = canCameraInteraface.attr("getJsYdemand")() ;
    //     yDemand = boost::python::extract<int>(  temp)();
    // } catch( boost::python::error_already_set ) {
    //     PyErr_Print();
    // }

    RsCamera rsCamera;
    ObjectDetector objectDetector(&rsCamera);
    Displayer display(&rsCamera);
    PCLViewer pclViewer;

          

    std::vector<std::vector<int>> obstacle_map( MAP_WIDTH, std::vector<int>(MAP_DEPTH, 0) );
    // std::vector<std::vector<std::pair<double, double>>> obstacle_map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr              xyzrgb_cloud(       new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 filteredCloud(      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 floor_area_cloud(   new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 obstacles_cloud(    new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 flattened_cloud(    new pcl::PointCloud<pcl::PointXYZ>);


    rs::log_to_console( rs::log_severity::warn );


    if( !rsCamera.startStreaming( ) )
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console( rs::log_severity::fatal );
        return EXIT_FAILURE;
    }

    std::pair<int, int> velocity_restrict{0,0};


    // display.initializeWindows();
    // pclViewer.addRGBCloud<pcl::PointXYZRGB>( xyzrgb_cloud,  "cloud",        1 );
    pclViewer.addXYZCloud<pcl::PointXYZ>( filteredCloud,    "orig",         1,  1,1,1 );
    pclViewer.addXYZCloud<pcl::PointXYZ>( floor_area_cloud, "floor",        4,  0,0,1 );
    pclViewer.addXYZCloud<pcl::PointXYZ>( obstacles_cloud,  "obstacles",    4,  1,0,0 );

    std::vector<std::pair<double, double>> arc;

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

        /* Filter point cloud to remove noise */
        objectDetector.applyPassThroughZ(filteredCloud, MAX_RANGE);
        objectDetector.removeOutliers(filteredCloud);

        /* Detect the floor plane */
        objectDetector.findFloor(filteredCloud, floor_area_cloud, obstacles_cloud);
        objectDetector.removeOutliers(obstacles_cloud);
        // objectDetector.removeOutliers(obstacles_cloud);
        objectDetector.flattenCloud(obstacles_cloud, flattened_cloud);
        objectDetector.clusterAndJoin2Dcloud(flattened_cloud, flattened_cloud );
        objectDetector.generateObstacleMap_grid<int>( obstacles_cloud, obstacle_map, MAP_WIDTH, MAP_DEPTH);


        /* Get the joystick demands from the CAN bus */
        // canCameraInteraface.attr("updateJsDemands")();
        // xDemand = boost::python::extract<int>(   canCameraInteraface.attr("getJsXdemand")()  )();
        // yDemand = boost::python::extract<int>(   canCameraInteraface.attr("getJsYdemand")()  )();

        canCameraInteraface.attr("update_speedDemands")();
        vel_linear    = 0.028 * boost::python::extract<int>(   canCameraInteraface.attr("get_speed_inear")()  )();
        vel_turn      = 0.028*3 *boost::python::extract<int>(   canCameraInteraface.attr("get_speed_turn")()  )();

        cout<<"linear: "<<vel_linear<<", turn: "<<vel_turn<<endl;

        // dt = rsCamera.getFps();
        dt=100;

        /* Modelling movement as an arc */
        if(vel_turn != 0)
        {
            theta = (vel_turn*(dt/100)/2);
            R = vel_linear/vel_turn *100;
            x = vel_linear * sin(theta)*100;
            y = vel_linear * cos(theta)*100;

            // x = R * sin(vel_turn*(dt/100));
            // y = R * cos(vel_turn*(dt/100));
            // theta = vel_turn / R;
            d = 2*R*tan(theta);
        }
        /* moving in a straight line */
        else 
        {
            theta = 0;
            d = vel_linear*(dt/100)*100;
            x = 0;
            y = d;
        }
        // R=200;
        // x=0;
        // y=0;
        // d=300;
        // theta=-M_PI/4;
        cout<<"x "<<x<<" y "<<y<<" R "<<R<<" d "<<d<<" theta "<<theta<<endl;
        // velocity_restrict = objectDetector.calculate_vel_restriction2(obstacle_map, v_linear, v_turn, dt);

        arc.clear();
        generateArc(R,2*theta,y,arc);

        /* Calculate the velocity adjustment/restriction needed */
        // velocity_restrict = objectDetector.calculate_vel_restriction(obstacle_map, xDemand, yDemand);

        // adjX = (velocity_restrict.first     > 0 )  ? (100 - velocity_restrict.first )  : (xDemand);
        // adjY = (velocity_restrict.second    > 0 && yDemand >0)  ? (100 - velocity_restrict.second)  : (yDemand);

        // adjX = (velocity_restrict.first     > 0 )  ? (0)  : (xDemand);
        // adjY = (velocity_restrict.second    > 0 )  ? (0)  : (yDemand);


        /* Send the adjusted velocity demands back onto the CAN bus */
        // canCameraInteraface.attr("sendAdjDemands")(adjX, adjY);

        



        /* ==== Display streams, clouds, plots ==== */
        display.displayFps(rsCamera.getFps());
        // cout<<"X: demand "<<xDemand<<", restrict "<<velocity_restrict.first<<", adj "<<adjX<<endl;
        // cout<<"Y: demand "<<yDemand<<", restrict "<<velocity_restrict.second<<", adj "<<adjY<<endl;



        std::vector<std::pair<double, double>> obstacle_plot;
        obstacle_plot.clear();
        for(int i = 0; i < MAP_WIDTH; i++)
        {
            for( int j=0; j<MAP_DEPTH; j++)
            {
                if( obstacle_map[i][j] == 1)
                {
                    obstacle_plot.push_back(std::pair<double, double> (i, j));
                }
            }
        }


        double js_mag   = K1*yDemand;
        double js_angle = K2*sin(xDemand*M_PI/200);


        pclViewer.clearPlot();
        pclViewer.plot( new (double [2]){0+MAP_WIDTH/2,x+MAP_WIDTH/2}, new (double [2]){0,  y}  );
        // pclViewer.plot( new (double [2]){0+MAP_WIDTH/2,js_mag*sin(js_angle)+MAP_WIDTH/2}, new (double [2]){0,  js_mag*cos(js_angle)}  );
        pclViewer.updatePlot(obstacle_plot);
        pclViewer.updatePlot(arc);

        // pclViewer.updatePlot(obstacle_map);
        pclViewer.display();
        // display.displayStreams();

    }

    rsCamera.stopStreaming();
    cv::destroyAllWindows( );
    Py_Finalize();
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
catch( boost::python::error_already_set )
{
    PyErr_Print();
}