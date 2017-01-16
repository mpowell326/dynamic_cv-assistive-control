#include "ObjectDetector.hpp"

#define CAMERA_HEIGHT   0.6 // Camera mounting offset from floor (m)
#define CAMERA_ANGLE    20.0  // Camera mounting angle (deg). Positive is rotated torwards ground.
#define VOXEL_LEAFSIZE  0.02f
#define FLOOR_ANGLE_EPS 10.0

ObjectDetector::ObjectDetector(){}
ObjectDetector::ObjectDetector(RsCamera* ptr)
{
	camera =  ptr;
}
void ObjectDetector::setCamera(RsCamera* ptr)
{
	camera =  ptr;
}




/* 	======================================================================
						Point Cloud Functions
	====================================================================== */
void ObjectDetector::applyVoxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, double leafsize)
{
	/* ==== Process/Filter the Point Cloud ==== */
	/* Downsample the cloud */
	pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
	voxgrid.setInputCloud(input);
	/* We set the size of every voxel to be 1x1x1cm
	 (only one point per every cubic centimeter will survive). */
	voxgrid.setLeafSize(leafsize, leafsize, leafsize);
	voxgrid.filter(*input);
}


void ObjectDetector::convertRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(*input, *output ) ;
}


void ObjectDetector::transform(pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{	
	/* ==== Transform point cloud to account for camera angle and offset ==== */ 
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	/* Define a translation of 2.5 meters on the x axis. */
	transform.translation() << 0.0, CAMERA_HEIGHT, 0.0;
	/* Rotate  around Z axis */
	transform.rotate (Eigen::AngleAxisf (pcl::deg2rad(CAMERA_ANGLE), Eigen::Vector3f::UnitX()));
	// Executing the transformation
	pcl::transformPointCloud (*orig_cloud, *transformed_cloud, transform);
}


void ObjectDetector::applyPassThroughZ(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double range)
{
	/* Pass through Filter */
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(input);
	/* Filter out all points with Z values not in the [0-4] range. */
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0.0, range);
	filter.filter(*input);
}


void ObjectDetector::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	/* Statistical Outlier Removal filter */
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (input);
	sor.setMeanK (100);
	sor.setStddevMulThresh (0.4);
	sor.filter (*input);
}


void ObjectDetector::findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud)
{
	/* ==== Floor Segmentation Setup ==== */
	pcl::ModelCoefficients::Ptr floor_coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr floor_inliers (new pcl::PointIndices);
	

	/* ==== Find the floor plane ====*/

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(25);
    seg.setDistanceThreshold (0.02); // 2 cm
    seg.setAxis(Eigen::Vector3f (0.0, 1.0, 0.0));
    seg.setEpsAngle(pcl::deg2rad(FLOOR_ANGLE_EPS));
    seg.setInputCloud(input);
    seg.segment (*floor_inliers, *floor_coefficients);

    if (floor_inliers->indices.size () != 0)
    {
        pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*input, *floor_inliers, *floor_cloud ) ;

        double a= floor_coefficients->values[0] ;
        double b= floor_coefficients->values[1] ;
        double c= floor_coefficients->values[2] ;
        double d= floor_coefficients->values[3] ;
        double floor_offset = d/b;
        double floor_angle = atan(c/b);


        /* ==== Extract floor plan and flatten remaining obstacles ==== */

        pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*input, *obstacles_cloud ) ;
        

        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // transform.translation() << floor_offset, 0.0, 0.0;
        // transform.rotate (Eigen::AngleAxisf (pcl::deg2rad(floor_angle), Eigen::Vector3f::UnitX()));
        // pcl::transformPointCloud (*obstacles_cloud, *obstacles_cloud, transform);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        // Extract the outliers
        extract.setInputCloud (obstacles_cloud);
        extract.setIndices (floor_inliers);
        extract.setNegative (true);
        extract.filter (*obstacles_cloud);

    }
    else
    {
        floor_cloud->clear();
        obstacles_cloud->clear();
    }

}


void ObjectDetector::flattenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud)
{
        pcl::ModelCoefficients::Ptr flatten_coefficients (new pcl::ModelCoefficients ());
        flatten_coefficients->values.resize (4);
        flatten_coefficients->values[1] = 1.0;
        flatten_coefficients->values[0] = flatten_coefficients->values[2] = flatten_coefficients->values[3] = 0;

        pcl::ProjectInliers<pcl::PointXYZ> projFlat;
        projFlat.setModelType (pcl::SACMODEL_PLANE);
        projFlat.setInputCloud (input);
        projFlat.setModelCoefficients (flatten_coefficients);
        projFlat.filter (*flattened_cloud);
}


void ObjectDetector::cluster2Dcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::vector<std::vector<std::pair<double, double>>> &plotData )
{
    /* ==== Eucludian Cluster to estimate obstacle positons ==== */
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.10); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input);
    ec.extract (cluster_indices);

    plotData.clear();
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          	cloud_cluster->points.push_back (input->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }

        //Generate data
        std::vector<std::pair<double, double>>  obstacle;
        generateObstacleDistancesVector( cloud_cluster, obstacle);    
        plotData.push_back(obstacle);   
    }
}



// void ObjectDetector::findObstacles(pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyxrgb_cloud, std::vector<std::pair<double, double>> osbtacle_map )
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr                 filteredCloud(  new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr                 floor_area_cloud(   new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr                 obstacles_cloud(    new pcl::PointCloud<pcl::PointXYZ>);

//      Transform input cloud to account for camera mounting 
// 	transform(xyxrgb_cloud,  xyxrgb_cloud);

// 	/* Filter point cloud to remove noise and downsample */
// 	convertRGBtoXYZ(xyxrgb_cloud, filteredCloud);
// 	applyVoxelGrid(filteredCloud, VOXEL_LEAFSIZE);
// 	applyPassThroughZ(filteredCloud, MAX_RANGE);
// 	removeOutliers(filteredCloud);

// 	/* Detect the floor plane */
// 	findFloor(filteredCloud, floor_area_cloud, obstacles_cloud);
// 	flattenCloud(obstacles_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud);
// 	cluster2dcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, osbtacle_map );
// }


//===================================================================
// Populate distance to obstacles vector given 2d map
//===================================================================
void ObjectDetector::generateObstacleDistancesVector( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::pair<double, double> >  &plot_data)
{
    int i;
    double angle, distance, x, z;
    plot_data.clear();

    for ( i=0; i<=cloud->points.size(); i++ )
    {
        x = cloud->points[i].x;
        z = cloud->points[i].z;

        angle = - atan2(x,z) * 360 /(2*M_PI);
        distance = sqrt(x*x + z*z);
        plot_data.push_back( std::pair<double, double> (angle, distance));
    }
}



























bool ObjectDetector::isObjectClose()
{
	// Mat depthFrame = (* camera->getDepthFrame() );
	Mat depthFrame = (* camera->getMat(rs::stream::depth));
	Mat output;

	const float distance = 200;


	//apply  thresholding
	cv::threshold(depthFrame, output, distance, distance, THRESH_TOZERO_INV);
	
	float mean = cv::mean(output)[0];
	cout<<mean<<endl;
	

	// convertRsFrame2Mat(&output,&output);

	// cv::imshow( "Thresholded", output );
	// waitKey(1);	


	/* Is object likely closer than 'distance = 2m' to camera. */
	if (mean > 20) 
	{
		return true;
	}
	else
	{
	return false;
	}
}


// void ObjectDetector::isWindowSafe()
// {
	
// }

void ObjectDetector::findFloor()
{
	Mat depthFrame = (* camera->getMat(rs::stream::depth));
	Mat infrared = (* camera->getMat(rs::stream::infrared));
	Mat output;
	Mat contours;

	// cv::dilate(depthFrame,output,cv::Mat(),cv::Point(-1,-1),6);
	cv::medianBlur(output, output, 13);
	cv::dilate(depthFrame, output, cv::Mat(), cv::Point(-1, -1), 5);
	// cv::erode(output, output, cv::Mat(), cv::Point(-1, -1), 9);
	cv::medianBlur(output, output, 3);


	// cv::Canny(infrared,contours,35,90);

	cv::imshow( "Median Blur", output );
	// cv::imshow( "Cany Edge Detector", output + contours );
	waitKey(1);	
}


void ObjectDetector::findSegments()
{
	// Mat depthFrame = (* camera->getDepthFrame());
	Mat depthFrame = (* camera->getMat(rs::stream::depth));
	Mat fg;
	cv::erode(depthFrame,fg,cv::Mat(),cv::Point(-1,-1),6);

	cv::Mat bg;
	cv::dilate(depthFrame,bg,cv::Mat(),cv::Point(-1,-1),6);
	cv::threshold(bg,bg,1,128,cv::THRESH_BINARY_INV);

	// Create markers image
	cv::Mat markers(depthFrame.size(),CV_8U,cv::Scalar(0));
	markers= fg+bg;

	// Create watershed segmentation object
	WatershedSegmenter segmenter;
	// Set markers and process
	segmenter.setMarkers(markers);
	// segmenter.process(markers);

	Mat tmp = segmenter.getSegmentation();
	// convertRsFrame2Mat(&tmp,&markers);

	// cv::imshow( "Segmented Image", markers );
	// waitKey(1);	
}


void ObjectDetector::findBlobs()
{
	// Read image
	// Mat depthFrame = (* camera->getDepthFrame());
	Mat depthFrame = (* camera->getMat(rs::stream::depth));
	Mat im;
	const float distance = 2;


	//apply otsu thresholding
	cv::threshold(depthFrame, im, distance, distance, THRESH_TOZERO_INV);
	// convertDepthMat4Display(&im,&im);

	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;

	// Filter by Area.
	params.filterByArea = false;
	// params.minArea = 1500;

	// Filter by Circularity
	params.filterByCircularity = false;
	// params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;

	// Filter by Inertia
	// params.filterByInertia = true;
	// params.minInertiaRatio = 0.01;



	// Set up the detector with default parameters.
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	 
	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	detector->detect( im, keypoints);
	cout<<"blobs"<<endl;
	cout<<keypoints.size() <<endl;
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	 
	// Show blobs
	
	imshow("keypoints", im_with_keypoints );
	waitKey(1);
}
