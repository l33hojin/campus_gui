#include "hprobot_arm_control.h"
#include "ui_hprobot_arm_control.h"
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
HProbotArmControl::HProbotArmControl(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::HProbotArmControl)
{
  ui->setupUi(this);
  n.reset(new ros::NodeHandle("~"));

  ui->stackedWidget->setCurrentIndex(0);

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(1);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate
  ros::AsyncSpinner spinner(4);
  spinner.start();
  static const std::string PLANNING_GROUP = "interbotix_arm";
  
  move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  QString text_log;
  text_log.sprintf("[INFO] [%lf] Hprobot Arm Control Node START",ros::Time::now().toSec());
  ui->textEdit_page2_execute_log->append(text_log);

  ROS_INFO_NAMED("moveit_interface", "Reference frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveit_interface", "End effector link: %s", move_group->getEndEffectorLink().c_str());


  /*visual_tools = new moveit_visual_tools::MoveItVisualTools(move_group->getPlanningFrame());
  visual_tools->deleteAllMarkers();
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1;
  visual_tools->publishText(text_pose, "hprobot_arm", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools->trigger();*/

 // manipulator_coordinate_pub_ = n->advertise<std_msgs::Float32>("/wlkata_coordinate",1000);

  //spinner.stop();
  
 //ros::Subscriber color_image_sub_;
  //color_image_sub_ = new ros::Subscriber;
  //color_image_sub_ = n->subscribe("/camera/depth/camera_info", 1, &HProbotArmControl::color_image_sub_cb, this);
}


void HProbotArmControl::spinOnce()
{
    if(ros::ok())
    {
        ros::spinOnce();
    }
    else
        QApplication::quit();
}

HProbotArmControl::~HProbotArmControl()
{
  ROS_INFO("HProbotArmControl Node SHUTDOWN");
  delete ui;
}


void HProbotArmControl::on_pushButton_page0_main_handeyecalibration_clicked()
{
  ui->stackedWidget->setCurrentIndex(1);
}


void HProbotArmControl::on_pushButton_page1_calibration_home_clicked()
{
  ui->stackedWidget->setCurrentIndex(0);
}


void HProbotArmControl::on_pushButton_page2_execute_home_clicked()
{
  ui->stackedWidget->setCurrentIndex(0);
}


void HProbotArmControl::on_pushButton_page0_main_execute_clicked()
{
  ui->stackedWidget->setCurrentIndex(2);
}


void HProbotArmControl::on_pushButton_page2_execute_detectmarker_clicked()
{
  QString text_log;
  text_log.sprintf("[INFO] [%lf] Service Call 'marker_detection' ",ros::Time::now().toSec());
  ui->textEdit_page2_execute_log->append(text_log);

  ros::ServiceClient client = n->serviceClient<hprobot_module::hprobot_marker_detector>("/vx300/marker_detection");
  hprobot_module::hprobot_marker_detector srv;
  if(client.call(srv))
  {
    ui->textEdit_page2_execute_log->setTextColor(QColor(0,0,255));
    text_log.sprintf("[INFO] [%lf] tvec : [%.2lf, %.2lf, %.2lf] ",ros::Time::now().toSec(),srv.response.tvec.data[0], srv.response.tvec.data[1], srv.response.tvec.data[2]);
    ui->textEdit_page2_execute_log->append(text_log);

    text_log.sprintf("[INFO] [%lf] rvec : [%.2lf, %.2lf, %.2lf] ",ros::Time::now().toSec(),srv.response.rvec.data[0], srv.response.rvec.data[1], srv.response.rvec.data[2]);
    ui->textEdit_page2_execute_log->append(text_log);
    ui->textEdit_page2_execute_log->setTextColor(QColor(0,0,0));

    double rvec[3];
    double tvec[3];

    for(int i=0 ; i<3 ; i++){
      rvec[i] = srv.response.rvec.data[i];
      tvec[i] = srv.response.tvec.data[i];
    }

    cv::Mat marker_tvec(3,1, CV_64FC1, tvec);
    cv::Mat marker_rvec_rod(3,1, CV_64FC1, rvec);
    cv::Mat marker_rvec;
    cv::Rodrigues(marker_rvec_rod, marker_rvec);

    cv::Mat T = cv::Mat::eye(4, 4, marker_rvec.type()); // T is 4x4
    T( cv::Range(0,3), cv::Range(0,3) ) = marker_rvec * 1; // copies R into T
    T( cv::Range(0,3), cv::Range(3,4) ) = marker_tvec * 1;

    marker2camera = T.clone();
  }
  else
  {
    ui->textEdit_page2_execute_log->setTextColor(QColor(255,0,0));
    text_log.sprintf("[INFO] [%lf] Service Call Failed",ros::Time::now().toSec());
    ui->textEdit_page2_execute_log->append(text_log);
    ui->textEdit_page2_execute_log->setTextColor(QColor(0,0,0));
  }

}


void HProbotArmControl::on_pushButton_page2_execute_generatecollisionobject_clicked()
{
  QString text_log;
  text_log.sprintf("[INFO] [%lf] Service Call 'collision generator' ",ros::Time::now().toSec());
  ui->textEdit_page2_execute_log->append(text_log);

  float position[3];
  float orientation[4];

  ros::AsyncSpinner spinner(4);
  spinner.start();

  position[0] = move_group->getCurrentPose().pose.position.x;
  position[1] = move_group->getCurrentPose().pose.position.y;
  position[2] = move_group->getCurrentPose().pose.position.z;

  orientation[0] = move_group->getCurrentPose().pose.orientation.w;
  orientation[1] = move_group->getCurrentPose().pose.orientation.x;
  orientation[2] = move_group->getCurrentPose().pose.orientation.y;
  orientation[3] = move_group->getCurrentPose().pose.orientation.z;

  spinner.stop();


  text_log.sprintf("[INFO] [%lf] Get Current Pose Success",ros::Time::now().toSec());
  ui->textEdit_page2_execute_log->append(text_log);

  text_log.sprintf("[INFO] [%lf] Position = [%.2lf, %.2lf, %.2lf]",ros::Time::now().toSec(), position[0], position[1], position[2]);
  ui->textEdit_page2_execute_log->append(text_log);

  text_log.sprintf("[INFO] [%lf] Orientation = [%.2lf, %.2lf, %.2lf, %.2lf]",ros::Time::now().toSec(), orientation[0], orientation[1], orientation[2], orientation[3]);
  ui->textEdit_page2_execute_log->append(text_log);


  tf::Quaternion q;

  q.setW(orientation[0]);
  q.setX(orientation[1]);
  q.setY(orientation[2]);
  q.setZ(orientation[3]);

  tf::Matrix3x3 rt(q);


  cv::Matx44f gripper2base = {
    (float)rt[0][0], (float)rt[0][1], (float)rt[0][2], position[0],
    (float)rt[1][0], (float)rt[1][1], (float)rt[1][2], position[1],
    (float)rt[2][0], (float)rt[2][1], (float)rt[2][2], position[2],
    0, 0, 0, 1};

  std::cout << marker2camera  << std::endl;
  std::cout << gripper2base << std::endl;

  //===================object 생성

  moveit_msgs::CollisionObject collision_object;
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose box_pose;
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  collision_object.operation = collision_object.ADD;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.id = "collision_obj";
  Eigen::Vector3d b(0.001, 0.001, 0.001);
  shapes::Mesh* m;
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  geometry_msgs::Pose mesh_pose;

  tf2::Quaternion calQuaternion;
  //===================robot 밑면========================//
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.4;
  //primitive.dimensions[2] = 0.4;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  -0.2;
  box_pose.position.y =  0.0;
  box_pose.position.z =  -0.025;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_objects.push_back(collision_object);

  //====================하판====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/1138.600.10.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.27;
  mesh_pose.position.y = 0.569;
  mesh_pose.position.z = 0.20;

  calQuaternion.setRPY(0,0,-1.57);
  calQuaternion=calQuaternion.normalize();

  mesh_pose.orientation.w= calQuaternion.getW();
  mesh_pose.orientation.x= calQuaternion.getX();;
  mesh_pose.orientation.y= calQuaternion.getY();;
  mesh_pose.orientation.z= calQuaternion.getZ();;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);

  //====================상판====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/1138.600.10.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.27;
  mesh_pose.position.y = 0.569;
  mesh_pose.position.z = 0.580;

  calQuaternion.setRPY(0,0,-1.57);
  calQuaternion=calQuaternion.normalize();

  mesh_pose.orientation.w= calQuaternion.getW();
  mesh_pose.orientation.x= calQuaternion.getX();;
  mesh_pose.orientation.y= calQuaternion.getY();;
  mesh_pose.orientation.z= calQuaternion.getZ();;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);

  //====================하판프레임====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/30.30.1138.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.240;
  mesh_pose.position.y = -0.569;
  mesh_pose.position.z = 0.18;
  mesh_pose.orientation.w= calQuaternion.getW();
  mesh_pose.orientation.x= calQuaternion.getX();;
  mesh_pose.orientation.y= calQuaternion.getY();;
  mesh_pose.orientation.z= calQuaternion.getZ();;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);
  //====================기둥 1====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/30.30.370.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.240;
  mesh_pose.position.y = 0.569;
  mesh_pose.position.z = 0.21;
  mesh_pose.orientation.w= 1.0;
  mesh_pose.orientation.x= 0.0;
  mesh_pose.orientation.y= 0.0;
  mesh_pose.orientation.z= 0.0;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);
  //====================기둥 2====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/30.30.370.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.240;
  mesh_pose.position.y = -0.599;
  mesh_pose.position.z = 0.21;
  mesh_pose.orientation.w= 1.0;
  mesh_pose.orientation.x= 0.0;
  mesh_pose.orientation.y= 0.0;
  mesh_pose.orientation.z= 0.0;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);
  //====================기둥 3====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/30.30.370.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.870;
  mesh_pose.position.y = 0.569;
  mesh_pose.position.z = 0.21;
  mesh_pose.orientation.w= 1.0;
  mesh_pose.orientation.x= 0.0;
  mesh_pose.orientation.y= 0.0;
  mesh_pose.orientation.z= 0.0;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);
  //====================기둥 4====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/30.30.370.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.870;
  mesh_pose.position.y = -0.599;
  mesh_pose.position.z = 0.21;
  mesh_pose.orientation.w= 1.0;
  mesh_pose.orientation.x= 0.0;
  mesh_pose.orientation.y= 0.0;
  mesh_pose.orientation.z= 0.0;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);

  //====================상판프레임====================//
  m = shapes::createMeshFromResource("file:///home/hprobot/simul_stl/30.30.1138.stl",b);
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  mesh_pose.position.x = 0.240;
  mesh_pose.position.y = -0.569;
  mesh_pose.position.z = 0.58;
  mesh_pose.orientation.w= calQuaternion.getW();
  mesh_pose.orientation.x= calQuaternion.getX();;
  mesh_pose.orientation.y= calQuaternion.getY();;
  mesh_pose.orientation.z= calQuaternion.getZ();;

  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(mesh_pose);
  collision_objects.push_back(collision_object);


  //==========================end==============================
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  sleep(2.0);

  /*ros::ServiceClient client = n->serviceClient<hprobot_module::hprobot_collision_generator>("/vx300/collision_generation");
  hprobot_module::hprobot_collision_generator srv;
  if(client.call(srv))
  {
    ROS_INFO("SUCCESS");
  }
  else
  {
    ROS_INFO("FAILED");
  }*/




}



void HProbotArmControl::on_pushButton_page1_show_image_clicked()
{
    ros::NodeHandle n_show_image;
    cv_bridge::CvImagePtr cv_ptr;

    double intrinsic_parameter[9];
    double discoeffs[4];

    sensor_msgs::ImageConstPtr image_raw;
    sensor_msgs::CameraInfoConstPtr camera_info;

    image_raw = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw",n_show_image);
    camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",n_show_image);

    for(int i=0;i<9;i++){
      intrinsic_parameter[i] = camera_info->K[i];
    }

    try {
      cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e){
      ROS_ERROR("Error!");
      return;
    }

    cv::Mat color_image = cv_ptr->image.clone();

    cv::cvtColor(color_image, color_image, CV_BGR2RGB);
    cv::resize(color_image, color_image, cv::Size(640, 320));
    ui->label_page1_color_image->setPixmap(QPixmap::fromImage(QImage(color_image.data, color_image.cols, color_image.rows, color_image.step, QImage::Format_RGB888)));
    n_show_image.shutdown();
}

void HProbotArmControl::on_pushButton_page1_detect_board_clicked()
{
    int squaresX = 8;//인쇄한 보드의 가로방향 마커 갯수
    int squaresY = 5;//인쇄한 보드의 세로방향 마커 갯수
    float squareLength = 30;//검은색 테두리 포함한 정사각형의 한변 길이, mm단위로 입력
    float markerLength = 23;//인쇄물에서의 마커 한변의 길이, mm단위로 입력
    int dictionaryId = 11;//DICT_6X6_250=10


    double intrinsic_parameter[9];
    double discoeffs[4];

    cv::Mat t2c_rvec = (cv::Mat_<float>(3, 3));
    cv::Mat t2c_tvec = (cv::Mat_<float>(3, 1));

    ros::NodeHandle n_show_image;
    cv_bridge::CvImagePtr cv_ptr;

    sensor_msgs::ImageConstPtr image_raw, depth_image_raw;
    sensor_msgs::CameraInfoConstPtr camera_info;

    image_raw       = ros::topic::waitForMessage<sensor_msgs::Image>     ("/camera/color/image_raw",n_show_image);
    depth_image_raw = ros::topic::waitForMessage<sensor_msgs::Image>     ("/camera/aligned_depth_to_color/image_raw",n_show_image);
    camera_info     = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",n_show_image);

    n_show_image.shutdown();

    for(int i=0;i<9;i++)
    {
      intrinsic_parameter[i] = camera_info->K[i];
    }

    RS_camera_info_.width = camera_info->width;    // Image Resoltusion width
    RS_camera_info_.height = camera_info->height;  // Image Resoltusion height
    RS_camera_info_.fx = camera_info->K[0];        // 초점거리 x
    RS_camera_info_.fy = camera_info->K[4];        // 초점거리 y
    RS_camera_info_.ppx = camera_info->K[2];       // 주점 x
    RS_camera_info_.ppy = camera_info->K[5];       // 주점 y
    RS_camera_info_.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY;

    for(int i=0;i<5;i++)
    {
      RS_camera_info_.coeffs[i] = camera_info->D[i];
    }

    cv::Mat A(3, 3, CV_64FC1, intrinsic_parameter);	// camera matrix
    cv::Mat distCoeffs(4, 1, CV_64FC1, discoeffs);

    try {
      cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e){
      ROS_ERROR("Error!");
      return;
    }
    cv::Mat color_image = cv_ptr->image.clone();

    cv::Mat depth_image;
    cv_ptr = cv_bridge::toCvCopy(depth_image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
    cv_ptr->image.convertTo(depth_image, CV_32F, 0.001);


    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);

    std::vector<cv::Point2f> chessboard_corners;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;

    cv::aruco::detectMarkers(color_image, board->dictionary, corners, ids, detectorParams, rejected);
    if (ids.size() > 0)
    {
      cv::aruco::drawDetectedMarkers(color_image, corners);
      std::vector<cv::Point2f> charucoCorners;
      std::vector<int> charucoIds;
      cv::aruco::interpolateCornersCharuco(corners, ids, color_image, board, charucoCorners, charucoIds);
      // if at least one charuco corner detected

      if (charucoIds.size() > 0)
      {
        cv::aruco::drawDetectedCornersCharuco(color_image, charucoCorners, charucoIds, cv::Scalar(255, 255, 0));
        bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, A, distCoeffs, t2c_rvec, t2c_tvec);
        //if (valid) cv::drawFrameAxes(color_image, A, distCoeffs, t2c_rvec, t2c_tvec, 100);

        std::vector<cv::Point3f> axesPoints;
        axesPoints.push_back(cv::Point3f(0, 0, 0));
        //axesPoints.push_back(cv::Point3f(100, 0, 0));
        //axesPoints.push_back(cv::Point3f(0,100, 0));
        //axesPoints.push_back(cv::Point3f(0, 0, 100));
        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(axesPoints, t2c_rvec, t2c_tvec, A, distCoeffs, imagePoints);


        float distance = depth_image.at<float>(imagePoints[0].y, imagePoints[0].x)*1000;
        float op_point[3];
        float pixel[2];

        pixel[0] = imagePoints[0].x;
        pixel[1] = imagePoints[0].y;

        rs2_deproject_pixel_to_point(op_point, &RS_camera_info_, pixel, distance);

        std::cout << op_point[0] << std::endl << op_point[1] << std::endl << op_point[2] << std::endl;

        //std::cout << pixel[0] << std::endl << pixel[1] << std::endl;
        //std::cout << " origin = " << t2c_tvec << std::endl;
        cv::Mat RR;
        cv::Rodrigues(t2c_rvec,RR);
        //t2c_tvec = -RR.inv() * t2c_tvec;

        /*t2c_tvec.at<float>(0,0) = op_point[0];
        t2c_tvec.at<float>(1,0) = op_point[1];
        t2c_tvec.at<float>(2,0) = op_point[2];*/

        //cv::Rodrigues(rvec, R);.
        std::cout << RR << std::endl;
        std::cout << t2c_tvec << std::endl;


      }
    }
    cv::Mat image_show = color_image.clone();
    cv::cvtColor(image_show, image_show, CV_BGR2RGB);
    cv::resize(image_show, image_show, cv::Size(640, 320));
    ui->label_page1_color_image->setPixmap(QPixmap::fromImage(QImage(image_show.data, image_show.cols, image_show.rows, image_show.step, QImage::Format_RGB888)));

    /*QString text_log;

    text_log.sprintf("[INFO] [%lf] Get Current Pose Success",ros::Time::now().toSec());
    ui->textEdit_page1_calibration_log->append(text_log);

    text_log.sprintf("[INFO] [%lf] Position = [%.2lf, %.2lf, %.2lf]",ros::Time::now().toSec(), position[0], position[1], position[2]);
    ui->textEdit_page1_calibration_log->append(text_log);

    text_log.sprintf("[INFO] [%lf] Orientation = [%.2lf, %.2lf, %.2lf, %.2lf]",ros::Time::now().toSec(), orientation[0], orientation[1], orientation[2], orientation[3]);
    ui->textEdit_page1_calibration_log->append(text_log);*/


}

void HProbotArmControl::on_pushButton_page1_get_match_clicked()
{

}

void HProbotArmControl::on_pushButton_page0_main_moving_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
    ui->lineEdit_page3_coordinate_x->clear();
    ui->lineEdit_page3_coordinate_y->clear();
    ui->lineEdit_page3_coordinate_z->clear();
}

void HProbotArmControl::on_pushButton_page3_home_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

void HProbotArmControl::on_pushButton_page3_execute_clicked()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();
    geometry_msgs::Pose coordinate;


    QString text_log;
    text_log.sprintf("[INFO] [%lf] Manipulator Moving...' ",ros::Time::now().toSec());
    ui->textEdit_page3_moving_log->append(text_log);

    coordinate.position.x = ui->lineEdit_page3_coordinate_x->text().toDouble();
    coordinate.position.y = ui->lineEdit_page3_coordinate_y->text().toDouble();
    coordinate.position.z = ui->lineEdit_page3_coordinate_z->text().toDouble();


    text_log.sprintf("[INFO] [%lf] Position = [%.2lf, %.2lf, %.2lf] ",ros::Time::now().toSec(),
                     coordinate.position.x, coordinate.position.y, coordinate.position.z);
    ui->textEdit_page3_moving_log->append(text_log);

    coordinate.orientation.w = 1;
    coordinate.orientation.x = 0;
    coordinate.orientation.y = 0;
    coordinate.orientation.z = 0;

    text_log.sprintf("[INFO] [%lf] orientation = [%.2lf, %.2lf, %.2lf, %.2lf] ",ros::Time::now().toSec(),
                     coordinate.orientation.w, coordinate.orientation.x, coordinate.orientation.y, coordinate.orientation.z);
    ui->textEdit_page3_moving_log->append(text_log);

    move_group->setStartState(*move_group->getCurrentState());
    move_group->setPoseTarget(coordinate);
    move_group->setGoalOrientationTolerance(0.1);
    bool success = (move_group->plan(saved_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        text_log.sprintf("[INFO] [%lf] Planning Success! Saved Plan Execute... ",ros::Time::now().toSec());
        ui->textEdit_page3_moving_log->append(text_log);
        move_group->execute(saved_plan);
    }
    else
    {
        text_log.sprintf("[INFO] [%lf] Failed. ",ros::Time::now().toSec());
        ui->textEdit_page3_moving_log->append(text_log);
    }


    text_log.sprintf("[INFO] [%lf] Finish! ",ros::Time::now().toSec());
    ui->textEdit_page3_moving_log->append(text_log);
    spinner.stop();
}
