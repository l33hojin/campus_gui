cmake_minimum_required(VERSION 3.0.2)
project(hprobot_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  image_transport
  sensor_msgs
  std_msgs
  cv_bridge
  geometry_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
  trajectory_msgs
  interbotix_xs_msgs
  moveit_commander
  moveit_ros_planning_interface
  moveit_visual_tools
)

find_package(OpenCV 4.2.0 REQUIRED)
find_package(realsense2 REQUIRED)

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

add_service_files(
    FILES
    hprobot_marker_detector.srv
    hprobot_collision_generator.srv
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
    CATKIN_DEPENDS message_runtime
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

add_executable(hprobot_marker_detector_server src/hprobot_marker_detector_server.cpp)
target_link_libraries(hprobot_marker_detector_server 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY})


add_executable(hptobot_collision_generater_server src/hptobot_collision_generater_server.cpp)
target_link_libraries(hptobot_collision_generater_server 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY})

#add_dependencies(hprobot_marker_detector_server hprobot_module_gencpp)
