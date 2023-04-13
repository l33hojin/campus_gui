#ifndef HPROBOT_ARM_CONTROL_H
#define HPROBOT_ARM_CONTROL_H


#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include <iostream>

// Hprobot module
#include <hprobot_module/hprobot_marker_detector.h>
#include <hprobot_module/hprobot_collision_generator.h>

// QT5
#include <QWidget>
#include <QFileDialog>
#include <QString>
#include <qtimer.h>
#include <QPixmap>
#include <QLabel>
#include <QMessageBox>
#include <QTextEdit>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <shape_msgs/Mesh.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometric_shapes/shape_operations.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>

#include <geometric_shapes/solid_primitive_dims.h>


// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <ros/topic.h>

#include <cv_bridge/cv_bridge.h>


namespace Ui {
class HProbotArmControl;
}

class HProbotArmControl : public QWidget
{
  Q_OBJECT

public:
  explicit HProbotArmControl(QWidget *parent = nullptr);
  ~HProbotArmControl();
  ros::Publisher manipulator_coordinate_pub_;

  ros::ServiceServer srv_moveit_plan;                                           // Service to plan or execute a goal pose for the end-effector
  Eigen::Isometry3d text_pose;                                                  // Pose of text w.r.t. the 'world' frame in Rviz
  const robot_state::JointModelGroup *joint_model_group;                        // Holds the joints in the 'interbotix_arm' group
  moveit_visual_tools::MoveItVisualTools *visual_tools;                         // Used to display text and other markers in Rviz
  moveit::planning_interface::MoveGroupInterface *move_group;                   // MoveIt object that can actually plan and execute trajectories
  moveit::planning_interface::MoveGroupInterface::Plan saved_plan;              // Plan object that holds the calculated trajectory
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  // Not applied in this demo but would be used to add objects to the world
  static const std::string PLANNING_GROUP;
  ros::NodeHandlePtr n;
  rs2_intrinsics RS_camera_info_;
  cv::Mat marker2camera;

private slots:
  void spinOnce();

  void on_pushButton_page0_main_handeyecalibration_clicked();

  void on_pushButton_page1_calibration_home_clicked();

  void on_pushButton_page2_execute_home_clicked();

  void on_pushButton_page0_main_execute_clicked();

  void on_pushButton_page2_execute_detectmarker_clicked();

  void on_pushButton_page2_execute_generatecollisionobject_clicked();

  void on_pushButton_page1_show_image_clicked();

  void on_pushButton_page1_detect_board_clicked();
  
  void on_pushButton_page1_get_match_clicked();

  void on_pushButton_page0_main_moving_clicked();

  void on_pushButton_page3_home_clicked();

  void on_pushButton_page3_execute_clicked();

private:
  Ui::HProbotArmControl *ui;
  QTimer *ros_timer;
};

#endif // HPROBOT_ARM_CONTROL_H
