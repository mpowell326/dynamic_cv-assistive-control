#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "RsCamera.hpp"
#include "ObjectDetector.hpp"
#include "common.hpp"





class CollisionDetector
{
public:
    CollisionDetector()
    :   rsCamera(), 
        objectDetector(&rsCamera),
        display(&rsCamera),
        xyzrgb_cloud(       new pcl::PointCloud<pcl::PointXYZRGB>),
        filteredCloud(      new pcl::PointCloud<pcl::PointXYZ>),
        floor_area_cloud(   new pcl::PointCloud<pcl::PointXYZ>),
        obstacles_cloud(    new pcl::PointCloud<pcl::PointXYZ>),
        flattened_cloud(    new pcl::PointCloud<pcl::PointXYZ>),
        obstacle_map( mapWidth, std::vector<int>( mapDepth, 0) ),
        obstacle_map_empty( mapWidth, std::vector<int>( mapDepth, 0) ) {}
    ~CollisionDetector();
    
    void start();
    void update();
    void stop();
    
    RsCamera rsCamera;
    ObjectDetector objectDetector;
    Displayer display;
    // PCLViewer pclViewer;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr              xyzrgb_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 filteredCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 floor_area_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 obstacles_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr                 flattened_cloud;

    bool isClose;
    // std::vector<std::vector<std::pair<double, double>>> obstacle_map;

    int mapWidth  = 400;
    int mapDepth  = 400;
    std::vector<std::vector<int>> obstacle_map;
    std::vector<std::vector<int>> obstacle_map_empty;
};



#endif  // COLLISION_DETECTOR_H