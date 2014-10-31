cmake_minimum_required(VERSION 2.8.3)
project(upperbody_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  /usr/local/include
)

link_directories (
  /usr/local/lib/
)

add_message_files(
  FILES
  upperbody.msg
  upperbodyArray.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable( upperbody_detector
  src/upperbody_detector.cpp
)

target_link_libraries( upperbody_detector
  ${catkin_LIBRARIES}
opencv_calib3d
opencv_contrib
opencv_core
opencv_features2d
opencv_flann
opencv_highgui
opencv_imgproc
opencv_legacy
opencv_ml
opencv_objdetect
opencv_photo
opencv_ts
opencv_video
opencv_videostab
)