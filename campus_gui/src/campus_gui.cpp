#include "campus_gui.h"
#include "ui_campus_gui.h"

campus_gui::campus_gui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::campus_gui)
{
  ui->setupUi(this);
  n.reset(new ros::NodeHandle("~"));

   ros_timer = new QTimer(this);
   connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
   ros_timer->start(1);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

   color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &campus_gui::color_image_sub_cb, this);
   depth_image_sub_ = n->subscribe("camera/aligned_depth_to_color/image_raw", 1000, &campus_gui::depth_image_sub_cb, this);
}

campus_gui::~campus_gui()
{
  delete ui;
}


void campus_gui::spinOnce()
{
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}


int campus_gui::movej(float *fTargetPos, float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType)
{
  ui->textEdit_log->append("Move_joint START!");
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
      ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>( "/dsr01m1509/motion/move_joint");



      dsr_msgs::MoveJoint srv;

      for(int i=0; i<6; i++)
          srv.request.pos[i] = fTargetPos[i];
      srv.request.vel = fTargetVel;
      srv.request.acc = fTargetAcc;
      srv.request.time = fTargetTime;
      srv.request.radius = fBlendingRadius;
      srv.request.mode = nMoveMode;
      srv.request.blendType = nBlendingType;
      srv.request.syncType = nSyncType;


      QString text_for_append;

      text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
      ui->textEdit_log->append(text_for_append);

      text_for_append.sprintf("  <vel> %7.3f , <acc> %7.3f, <time> %7.3f",srv.request.vel, srv.request.acc, srv.request.time);
      ui->textEdit_log->append(text_for_append);

      text_for_append.sprintf("  <mode> %d , <radius> %7.3f, <blendType> %d",srv.request.mode, srv.request.radius, srv.request.blendType);
      ui->textEdit_log->append(text_for_append);

      if(srvMoveJoint.call(srv))
      {
         text_for_append.sprintf("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
          ui->textEdit_log->append(text_for_append);
          return (srv.response.success);
      }
      else
      {        
           ui->textEdit_log->append("Failed to call service dr_control_service : move_joint");
          ros::shutdown();
          return -1;
      }

      return 0;

}




int campus_gui::movel(float *fTargetPos, float *fTargetVel, float *fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)
{
      ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
      ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>( "/dsr01m1509/motion/move_line");
      dsr_msgs::MoveLine srv;

      for(int i=0; i<6; i++)
          srv.request.pos[i] = fTargetPos[i];
      for(int i=0; i<2; i++){
          srv.request.vel[i] = fTargetVel[i];
          srv.request.acc[i] = fTargetAcc[i];
      }
      srv.request.time = fTargetTime;
      srv.request.radius = fBlendingRadius;
      srv.request.ref  = nMoveReference;
      srv.request.mode = nMoveMode;
      srv.request.blendType = nBlendingType;
      srv.request.syncType = nSyncType;

      ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
      ROS_INFO("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
      ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);

      if(srvMoveLine.call(srv))
      {
          ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
          return (srv.response.success);
      }
      else
      {
          ROS_ERROR("Failed to call service dr_control_service : move_line\n");
          ros::shutdown();
          return -1;
      }
      return 0;
}


void campus_gui::color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
  color_image = cv_ptr->image.clone();
  cv::resize(color_image, color_image, cv::Size(640, 480));
  cvtColor(color_image, color_image, cv::COLOR_BGR2RGB);
  ui->label_pic->setPixmap(QPixmap::fromImage(QImage(color_image.data, color_image.cols, color_image.rows, color_image.step, QImage::Format_RGB888)));
}


void campus_gui::depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
  depth_image = cv_ptr->image.clone();
  cv::resize(depth_image, depth_image, cv::Size(640, 480));
  //cvtColor(color_image, color_image, cv::COLOR_BGR2RGB);
  //ui->label_pic->setPixmap(QPixmap::fromImage(QImage(color_image.data, color_image.cols, color_image.rows, color_image.step, QImage::Format_RGB16)));
}


void campus_gui::on_Calubration_Start_BT_clicked()
{
  float targetpos[6] = {90,0,90,0,90,0};
  float velx[2] = {0,0};
  float accx[2] = {0,0};

  movej(targetpos, 0, 0, 3,0,0,0,0);

 float mid_targetpos[6] = {350, 350, 450, 0, -180.0, 180.0};

 movel(mid_targetpos,velx,accx,1,0,0,0,0,0);

  ui->textEdit_log->append("Calibration Start! ");
  ui->textEdit_log->append("Now Loading...");
}
