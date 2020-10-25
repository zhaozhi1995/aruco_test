#include <iostream>  
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

ros::Publisher imgmarkerShowPub;
ros::Publisher markerposePub;
cv::Mat cameraMatrix;
cv::Mat distCoeffs;

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
		ROS_DEBUG("find %lu aruco markers", ids.size());
		cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
		sensor_msgs::ImagePtr marker_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg();
		imgmarkerShowPub.publish(*marker_show_msg);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        // draw axis for each marker
        for(int i=0; i<ids.size(); i++)
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
		sensor_msgs::ImagePtr marker_pose_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCopy).toImageMsg();
		markerposePub.publish(*marker_pose_msg);
	} else {
		ROS_DEBUG("No aruco marker be found!", ids.size());
		sensor_msgs::ImagePtr marker_show_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		imgmarkerShowPub.publish(*marker_show_msg);
	}
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["cameraMatrix"] >> cameraMatrix;
    fsSettings["distCoeffs"] >> distCoeffs;

    fsSettings.release();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "aruco_marker_dect");
	ros::NodeHandle nh;

    if(argc != 2)
    {
        printf("please intput: rosrun aruco_test aruco_test [config file] \n"
               "for example: rosrun aruco_test aruco_test "
               "~/catkin_ws/src/aruco_test/config/camera.yaml \n");
        return 1;
    }

    string camera_conf = argv[1];
    ROS_INFO("camera_conf: %s", argv[1]);

    readParameters(camera_conf);

	ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/image/raw", 1, imageHandler);
	imgmarkerShowPub = nh.advertise<sensor_msgs::Image>("/image/marker_show", 1);
	markerposePub = nh.advertise<sensor_msgs::Image>("/image/marker_pose_estimate", 1);

	ros::spin();
    return 0;
}

