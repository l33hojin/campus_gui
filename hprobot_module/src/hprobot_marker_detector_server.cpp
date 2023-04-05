#include "ros/ros.h"
#include "hprobot_module/hprobot_marker_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/aruco/charuco.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/aruco/dictionary.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/topic.h>

#include <cv_bridge/cv_bridge.h>


class marker_detector
{
  public:

  marker_detector()
  {
    //service = n.advertiseService("tes);
    service = n.advertiseService("marker_detection", &marker_detector::marker_detection, this);
    ROS_INFO("marker_detector service Start");
  }

  ~marker_detector()
  {
    ROS_INFO("marker_detector Node STOP!");
  }


  public:

  ros::NodeHandle n;
  ros::ServiceServer service;

  sensor_msgs::ImageConstPtr image_raw;
  sensor_msgs::CameraInfoConstPtr camera_info;

  cv_bridge::CvImagePtr cv_ptr;


  double intrinsic_parameter[9];
  double discoeffs[4];



  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  std::vector<int> markerIds;


  bool marker_detection(hprobot_module::hprobot_marker_detector::Request &req, hprobot_module::hprobot_marker_detector::Response&res)
  {

    image_raw = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw",n);
    camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",n);

    for(int i=0;i<9;i++){
      intrinsic_parameter[i] = camera_info->K[i];
    }

    try {
      cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e){
      ROS_ERROR("Error!");
      return 0;
    }
    
    cv::Mat color_image = cv_ptr->image.clone();
    detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    float markerLength = 0.05;
    cv::Mat cameraMatrix(3, 3, CV_64FC1, intrinsic_parameter);	
    cv::Mat distCoeffs(4, 1, CV_64FC1, discoeffs);
    cv::Mat objPoints(4, 1, CV_32FC3);

    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, markerLength, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength, markerLength, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, 0, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(0, 0, 0);

    if (markerIds.size() > 0)
    {
      cv::aruco::drawDetectedMarkers(color_image, markerCorners, markerIds);
      int nMarkers = markerCorners.size();
      std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
      for (int i = 0; i < nMarkers; i++) 
      {
        solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        for(int j = 0; j < 3 ; j++){
          res.rvec.data.push_back(rvecs.at(i).val[j]);
          res.tvec.data.push_back(tvecs.at(i).val[j]);
        }
      }
    }
    return true;
  }

  private:
};


//bool marker_detector(hprobot_ros_arm::hprobot)
int main(int argc, char **argv)
{
  ros::init(argc, argv, "server");

  marker_detector md;
  ros::spin();

  return 0;
}
