#ifndef CAMPUS_GUI_H
#define CAMPUS_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJoint.h>
#include <QFileDialog>
#include <QString>
#include <qtimer.h>
#include <QPixmap>
#include <QLabel>

namespace Ui {
class campus_gui;
}

class campus_gui : public QWidget
{
  Q_OBJECT

public:
  explicit campus_gui(QWidget *parent = nullptr);

  ros::NodeHandlePtr n;
  ros::Subscriber color_image_sub_;
  ros::Subscriber depth_image_sub_;


  int movel(float fTargetPos[6], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);
  int movej(float fTargetPos[6], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType);

  void color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
  void depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);


  cv::Mat color_image;
  cv::Mat depth_image;

  ~campus_gui();

private slots:
  void spinOnce();
  void on_Calubration_Start_BT_clicked();

private:

  Ui::campus_gui *ui;
  QTimer *ros_timer;

};

#endif // CAMPUS_GUI_H
