#include "ObjectDetector.hpp"

ObjectDetector::ObjectDetector(){}
ObjectDetector::ObjectDetector(RsCamera* ptr)
{
	camera =  ptr;
}
void ObjectDetector::setCamera(RsCamera* ptr)
{
	camera =  ptr;
}

// bool ObjectDetector::isObscured()
// {
// 	Mat depthFrame = (* camera->getMat(rs::stream::depth));

// }


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
