
#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include "RsCamera.hpp"
#include "common.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>


using namespace std;
using namespace cv;

class ObjectDetector : public RsCamera
{
public:
	ObjectDetector();
    ObjectDetector(RsCamera * ptr);

    void setCamera(RsCamera* ptr);
    
    /* Point Cloud Processign functions */
	void applyVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafsize);
	void convertRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
	void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);
	void applyPassThroughZ(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double range);
	void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
	void findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud);
	void flattenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud);
	void cluster2Dcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::vector<std::vector<std::pair<double, double>>> &plotData );
	void clusterAndJoin2Dcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
	

	void generateObstacleMap_cart( 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::pair<double, double> >  &plot_data);
	void generateObstacleMap_rad( 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::pair<int, int> >  &plot_data);
	template <typename T> void generateObstacleMap_grid( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector <std::vector<T> >& obstacle_map, int map_width, int map_depth);
	int scanline( std::vector <std::vector<int> >& obstacle_map, int x, int z, int dx, int dz, int step_x, int step_z, int max_distance);
	int checkCollision( std::vector <std::vector<int> >& obstacle_map, double angle, double max_distance);
	std::pair<int, int> calculate_vel_restriction(std::vector <std::vector<int> >& obstacle_map, int jsXdemand, int jsYdemand);
	std::pair<int, int> calculate_vel_restriction2(std::vector <std::vector<int> >& obstacle_map, double vel_linear, double vel_turn, int dt);

	int y_restrict=0;


	/* Image processing Function */
    bool isObjectClose();
    void findSegments();
    void findBlobs();
    void findFloor();
    
private:
	RsCamera*  camera;
	// cv::SimpleBlobDetector blobDetector;
};


// struct Point{
// 	int x;
// 	int y;
// 	bool obstacle;
// };

// struct Map {
// 	int width;
// 	int height;

// 	Point* grid[width][height];
// }












class WatershedSegmenter {
private:
	cv::Mat markers;
	
public:
	void setMarkers(const cv::Mat& markerImage)
	{
		// Convert to image of ints
		markerImage.convertTo(markers,CV_32S);
	}
	
	cv::Mat process(const cv::Mat &image)
	{
		// Apply watershed
		cv::watershed(image,markers);
		return markers;
	
	}
	// Return result in the form of an image
	cv::Mat getSegmentation()
	{
		cv::Mat tmp;
		// all segment with label higher than 255
		// will be assigned value 255
		markers.convertTo(tmp,CV_8U);
		return tmp;
	}
	// Return watershed in the form of an image
	cv::Mat getWatersheds()
	{
		cv::Mat tmp;
		// Each pixel p is transformed into
		// 255p+255 before conversion
		markers.convertTo(tmp,CV_8U,255,255);
		return tmp;
	}
};


#endif  // OBJECT_DETECTOR_H