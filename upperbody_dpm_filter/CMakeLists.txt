cmake_minimum_required(VERSION 2.8.3)
project(upperbody_dpm_filter)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  ground_based_detector
  object_recognition_by_parts
)

find_package( PCL REQUIRED )

include_directories(
	${catkin_INCLUDE_DIRS}
	include
)

catkin_package()

add_executable( upperbody_dpm_filter
  	src/upperbody_dpm_filter.cpp
)

target_link_libraries( upperbody_dpm_filter
	${catkin_LIBRARIES}
)
