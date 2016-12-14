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

bool ObjectDetector::isObjectClose()
{
	Mat depthFrame = (* camera->getDepthFrame() );
	Mat output;

	const float distance = 1.2;


	//apply otsu thresholding
	cv::threshold(depthFrame, output, distance, distance, THRESH_TOZERO_INV);
	
	// cv::erode(output,output,Mat(),Point(-1,-1),3);
	// cv::dilate(output,output,Mat(),Point(-1,-1),3);

	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	// blobDetector.detect( depthFrame, keypoints);
	 
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	// drawKeypoints( depthFrame, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	 
	// Show blobs
	// imshow("keypoints", im_with_keypoints );

	cout<<cv::mean(output)[0]<<endl;
	convertDepthMat4Display(&output,&output);

	cv::imshow( "RGB Image", output );
	waitKey(1);	


	/* Is object likely closer than 'distance = 1.25m' to camera. */
	if (cv::mean(output)[0] > 0.1) 
	{
		return true;
	}
	else
	{
	return false;
	}
}