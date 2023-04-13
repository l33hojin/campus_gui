#include "ros/ros.h"
#include "hprobot_module/hprobot_collision_generator.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Mesh.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
class collision_generator
{
  public:

  collision_generator()
  {
    service = n.advertiseService("collision_generation", &collision_generator::collision_generation, this);
    ROS_INFO("collision_generator service Start");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    static const std::string PLANNING_GROUP = "interbotix_arm";
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //visual_tools = new moveit_visual_tools::MoveItVisualTools(move_group->getPlanningFrame());
    //visual_tools->deleteAllMarkers();
    //text_pose = Eigen::Isometry3d::Identity();
    //text_pose.translation().z() = 1;
    //visual_tools->publishText(text_pose, "hprobot_arm", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    //visual_tools->trigger();

    collision_object.operation = collision_object.ADD;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "collision_obj";    

    
  }

  ~collision_generator()
  {
    ROS_INFO("collision_generator Node STOP!");
  }


  public:

  ros::NodeHandle n;
  ros::ServiceServer service;

  moveit::planning_interface::MoveGroupInterface::Plan saved_plan;              // Plan object that holds the calculated trajectory
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  // Not applied in this demo but would be used to add objects to the world
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit_msgs::CollisionObject collision_object;
  shape_msgs::SolidPrimitive primitive;
  geometry_msgs::Pose box_pose;
  std::vector<moveit_msgs::CollisionObject> collision_objects;


  Eigen::Isometry3d text_pose;                                                  // Pose of text w.r.t. the 'world' frame in Rviz
  const robot_state::JointModelGroup *joint_model_group;                        // Holds the joints in the 'interbotix_arm' group
  moveit_visual_tools::MoveItVisualTools *visual_tools;                         // Used to display text and other markers in Rviz


  bool collision_generation(hprobot_module::hprobot_collision_generator::Request &req, hprobot_module::hprobot_collision_generator::Response&res)
  {
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
    return true;
  }

  private:
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "server");

  collision_generator cg;
  ros::spin();

  return 0;
}
