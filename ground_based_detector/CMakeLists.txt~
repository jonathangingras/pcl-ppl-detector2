cmake_minimum_required(VERSION 2.8.3)
project(ground_based_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  pcl_msgs
  message_generation
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

add_message_files(
  FILES
  human.msg
  humanArray.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

add_executable( ground_based_detector
	src/ground_based_detector.cpp
)

target_link_libraries( ground_based_detector
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
	pcl_people
)