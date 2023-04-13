#include <ros/ros.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Mesh.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Float32.h>

#include <geometric_shapes/shape_operations.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include "ros/ros.h"

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
#include <geometry_msgs/PointStamped.h>
#include <ros/topic.h>

#include <cv_bridge/cv_bridge.h>

namespace {
const char* about = "Create an ArUco marker image";
const char* keys  =
        "{@outfile |<none> | Output image }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{id       |       | Marker id in the dictionary }"
        "{ms       | 200   | Marker size in pixels }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}


class image_processing
{
  public:

  image_processing()
  {
    ROS_INFO("image_processing Node Start!");
    
    //color_image_sub_ = n.subscribe("/camera/color/image_raw", 1000, &image_processing::color_image_sub_cb, this);
    //color_camera_info_sub_ = n.subscribe("/camera/color/camera_info", 1000, &image_processing::color_camera_info_sub_cb, this);
  }

  ~image_processing()
  {
    ROS_INFO("image_processing Node STOP!");
  }


 /* void color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e){
      ROS_ERROR("Error!");
      return;
    }
    
    cv::Mat color_image = cv_ptr->image.clone();

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> markerIds;
                
    detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat outputImage = color_image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    float markerLength = 0.05;
    cv::Mat cameraMatrix(3, 3, CV_64FC1, intrinsic_parameter);	
    cv::Mat distCoeffs(4, 1, CV_64FC1, discoeffs);
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, markerLength, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(￩markerLength, markerLength, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, 0, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(0, 0, 0);

    // You can read camera parameters from tutorial_camera_params.yml
    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        int nMarkers = markerCorners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // Calculate pose for each marker

        cv::Point center;
        
        for (int i = 0; i < nMarkers; i++) {
          solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
          center.x = markerCorners.at(i).data()->x;
          center.y = markerCorners.at(i).data()->y;
          cv::putText(outputImage, cv::format("(%.2f,%.2f)", markerCorners.at(i).data()->x, markerCorners.at(i).data()->y),center, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 0);



          std::vector<cv::Point3f> plants;
          cv::Point3f plant;
          plant.x = 0.1;
          plant.y = 0.1;
          plant.z = 0.1;
          plants.push_back(plant);

          std::vector<cv::Point2f> plants_2d;
          cv::projectPoints(plants, rvecs.at(i),tvecs.at(i),cameraMatrix,distCoeffs,plants_2d);
          cv::circle(outputImage, plants_2d.at(i), 1, cv::Scalar(0,0,100), 3, cv::LINE_AA);
          cv::putText(outputImage,"Grip Point",plants_2d.at(i), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 0);

          std::cout << plants_2d.at(i).x << " " << plants_2d.at(i).y<< std::endl;
        }
        // Draw axis for each marker
        for(unsigned int i = 0; i < markerIds.size(); i++) {
            cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
    // Set coordinate system
    }
    // Calculate pose for each marker
    /*for (int i = 0; i < nMarkers; i++) {
    solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
    }

    cv::imshow("aruco marker",outputImage);
    cv::waitKey(1);
  }

*/
  public:
  ros::NodeHandle n;
  //ros::Publisher object_camera_2D_coordinate_pub_;
  //geometry_msgs::PointStamped object_camera_2D_coordinate_output;

  private:

  
  /*ros::Subscriber color_image_sub_;
  ros::Subscriber color_camera_info_sub_;*/

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_processing");

  image_processing ip;  

  sensor_msgs::ImageConstPtr color_image_raw;
  sensor_msgs::CameraInfoConstPtr camera_info;

  double intrinsic_parameter[9];
  double discoeffs[4];

  color_image_raw = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw",ip.n);
  camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ip.n);


  for(int i=0;i<9;i++)
  {
    intrinsic_parameter[i] = camera_info->K[i];
    std::cout << intrinsic_parameter[i] << std::endl;
  }


  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(color_image_raw, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e){
    ROS_ERROR("Error!");
    return 0;
  }
  
  cv::Mat color_image = cv_ptr->image.clone();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  std::vector<int> markerIds;
              
  detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  cv::Mat outputImage = color_image.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  float markerLength = 0.05;
  cv::Mat cameraMatrix(3, 3, CV_64FC1, intrinsic_parameter);	
  cv::Mat distCoeffs(4, 1, CV_64FC1, discoeffs);
  cv::Mat objPoints(4, 1, CV_32FC3);
  objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(0, markerLength, 0);
  objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength, markerLength, 0);
  objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength, 0, 0);
  objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(0, 0, 0);

  // You can read camera parameters from tutorial_camera_params.yml
  if (markerIds.size() > 0)
  {
      cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
      int nMarkers = markerCorners.size();
      std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
      // Calculate pose for each marker

      cv::Point center;
      
      for (int i = 0; i < nMarkers; i++) {
        solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        center.x = markerCorners.at(i).data()->x;
        center.y = markerCorners.at(i).data()->y;
        cv::putText(outputImage, cv::format("(%.2f,%.2f)", markerCorners.at(i).data()->x, markerCorners.at(i).data()->y),center, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 0);

        std::cout << "rvec =  "<< rvecs.at(i).val[0] << std::endl;
        std::cout << "rvec =  "<< rvecs.at(i).val[1] << std::endl;
        std::cout << "rvec =  "<< rvecs.at(i).val[2] << std::endl;
        std::cout << "tvec =  "<< tvecs.at(i).val[0] << std::endl;
        std::cout << "tvec =  "<< tvecs.at(i).val[1] << std::endl;
        std::cout << "tvec =  "<< tvecs.at(i).val[2] << std::endl;
        
        /*std::vector<cv::Point3f> plants;
        cv::Point3f plant;
        plant.x = 0.1;
        plant.y = 0.1;
        plant.z = 0.1;
        plants.push_back(plant);

        std::vector<cv::Point2f> plants_2d;
        cv::projectPoints(plants, rvecs.at(i),tvecs.at(i),cameraMatrix,distCoeffs,plants_2d);
        cv::circle(outputImage, plants_2d.at(i), 1, cv::Scalar(0,0,100), 3, cv::LINE_AA);
        cv::putText(outputImage,"Grip Point",plants_2d.at(i), cv::FONT_HERSHEY_DUPLEX, 2, cv::Scalar(0, 255, 0), 0);

        std::cout << plants_2d.at(i).x << " " << plants_2d.at(i).y<< std::endl;*/
      }
      // Draw axis for each marker
      for(unsigned int i = 0; i < markerIds.size(); i++) {
          cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
      }
  // Set coordinate system
  }
  // Calculate pose for each marker
  /*for (int i = 0; i < nMarkers; i++) {
  solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
  }*/

  cv::imshow("aruco marker",outputImage);
  //cv::waitKey(0);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  static const std::string PLANNING_GROUP = "interbotix_arm";

  ros::ServiceServer srv_moveit_plan;                                           // Service to plan or execute a goal pose for the end-effector
  Eigen::Isometry3d text_pose;                                                  // Pose of text w.r.t. the 'world' frame in Rviz
  const robot_state::JointModelGroup *joint_model_group;                        // Holds the joints in the 'interbotix_arm' group
  moveit_visual_tools::MoveItVisualTools *visual_tools;                         // Used to display text and other markers in Rviz
  moveit::planning_interface::MoveGroupInterface *move_group;                   // MoveIt object that can actually plan and execute trajectories
  moveit::planning_interface::MoveGroupInterface::Plan saved_plan;              // Plan object that holds the calculated trajectory
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  // Not applied in this demo but would be used to add objects to the world
  
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  visual_tools = new moveit_visual_tools::MoveItVisualTools(move_group->getPlanningFrame());
  visual_tools->deleteAllMarkers();
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1;
  visual_tools->publishText(text_pose, "hprobot_arm", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->trigger();

  //ros::spin();


  return 0;
}


