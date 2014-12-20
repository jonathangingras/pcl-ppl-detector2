cmake_minimum_required(VERSION 2.8.3)
project(upperbody_cropper)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  pcl_msgs
  message_generation
  ground_based_detector
)

find_package( PCL REQUIRED )

include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	.
	include
	/usr/include/pcl-1.7
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

add_executable( upperbody_cropper
	src/upperbody_cropper.cpp
)

target_link_libraries( upperbody_cropper
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	pcl_people
)
