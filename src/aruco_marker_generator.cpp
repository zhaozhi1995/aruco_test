#include <iostream>  
#include <string>  
#include <map>  
#include <ros/ros.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

//#define val_to_str(val)  #val
////eg: cv::aruco::DICT_6X6_250
//#define _DICT_nXn_s(n, s)  DICT_##n##X##n##_##s
//#define DICT_nXn_s(n, s) _DICT_nXn_s(n, s)

int marker_id;
int rowXcolumn;
int marker_size;
int pixel;
string marker_save_dir;

map<std::string, int> DICT_map = {
               { "DICT_4X4_50", cv::aruco::DICT_4X4_50 },
               { "DICT_4X4_100", cv::aruco::DICT_4X4_100 },
               { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
               { "DICT_4X4_1000", cv::aruco::DICT_4X4_1000 },
               { "DICT_5X5_50", cv::aruco::DICT_5X5_50 },
               { "DICT_5X5_100", cv::aruco::DICT_5X5_100 },
               { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
               { "DICT_5X5_1000", cv::aruco::DICT_5X5_1000 },
               { "DICT_6X6_50", cv::aruco::DICT_6X6_50 },
               { "DICT_6X6_100", cv::aruco::DICT_6X6_100 },
               { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
               { "DICT_6X6_1000", cv::aruco::DICT_6X6_1000 },
               { "DICT_7X7_50", cv::aruco::DICT_7X7_50 },
               { "DICT_7X7_100", cv::aruco::DICT_7X7_100 },
               { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
               { "DICT_7X7_1000", cv::aruco::DICT_7X7_1000 },
               { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL },
               { "DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 },
               { "DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9 },
               { "DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10 },
               { "DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 }
};

void aruco_marker_gengerate(int DICT, int id, const string& save_dir)
{
	char marker_image_name[32];
	sprintf(marker_image_name, "aruco_marker_%d_%dX%d_%d.jpg", marker_id, rowXcolumn, rowXcolumn, marker_size);
	// to gengerate a new marker
	cv::Mat markerImage;
	//cv::Ptr<cv::aruco::Dictionary> mdictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::Dictionary> mdictionary = cv::aruco::getPredefinedDictionary(DICT);
	cv::aruco::drawMarker(mdictionary, id, pixel, markerImage, 1);

	imshow("aruco marker image", markerImage);//œ‘ æmarker

	waitKey();
	imwrite(marker_save_dir + marker_image_name, markerImage);
	ROS_INFO("%s saved in %s", marker_image_name, marker_save_dir.c_str());

	cv::destroyAllWindows();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "aruco_marker_generator");
	ros::NodeHandle nh("~");

	ROS_INFO("read specitied marker property:");
	nh.param<int>("marker_id", marker_id, 0);
	ROS_INFO("marker id: %d", marker_id);
	nh.param<int>("rowXcolumn", rowXcolumn, 4);
	ROS_INFO("number of blocks in per row and column: %d", rowXcolumn);
	nh.param<int>("marker_size", marker_size, 100);
	ROS_INFO("marker length of a side: %d mm", marker_size);
	nh.param<int>("pixel", pixel, 500);
	ROS_INFO("pixel: %d", pixel);
	// if marker_DICT be set,then use DICT, rowXcolumn & marker_size will be no use. 
	// marker_DICT eg: DICT_4X4_100
	string _marker_DICT;
	nh.param<std::string>("marker_DICT", _marker_DICT, "");
	nh.param<std::string>("marker_save_dir", marker_save_dir, "./");
	ROS_INFO("marker image will be saved in %s", marker_save_dir.c_str());

	char marker_DICT[32];
	memset(marker_DICT, '\0', sizeof(marker_DICT));
	if (_marker_DICT.empty()) {
		sprintf((char *)marker_DICT, "DICT_%dX%d_%d", rowXcolumn, rowXcolumn, marker_size);
	} else {
		ROS_INFO("marker_DICT be set to: %s", _marker_DICT.c_str());
		strcpy(marker_DICT, _marker_DICT.c_str());
	}

	ROS_DEBUG("marker DICT %s, %d", marker_DICT, DICT_map.at(marker_DICT));

	aruco_marker_gengerate(DICT_map.at(marker_DICT), marker_id, marker_save_dir);

	//ros::spin();
    return 0;
}

