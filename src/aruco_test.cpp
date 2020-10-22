// aruco_code_detect.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//#include "pch.h"
#include <iostream>  
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/aruco/charuco.hpp>
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

ros::Publisher imgmarkerShowPub;

void imageHandler(const sensor_msgs::Image::ConstPtr& imageData) 
{
	cv::Mat image = cv_bridge::toCvCopy(imageData, "bgr8")->image;

	//cv::waitKey(30)
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

	cv::Ptr<cv::aruco::DetectorParameters> params = aruco::DetectorParameters::create();
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

	cv::Mat imageCopy;

	image.copyTo(imageCopy);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners, rejected;
	cv::aruco::detectMarkers(image, dictionary, corners, ids, params);
	
	// if at least one marker detected
	if (ids.size() > 0) 
	{
		ROS_INFO("find %lu aruco markers", ids.size());
		cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
	}

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg();
	imgmarkerShowPub.publish(*msg);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "aruco_marker_dect");
	ros::NodeHandle nh;

	ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/image/raw", 1, imageHandler);
	imgmarkerShowPub = nh.advertise<sensor_msgs::Image>("/image/marker_show", 1);

	ros::spin();
    return 0;
}

