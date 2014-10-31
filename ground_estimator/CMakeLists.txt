cmake_minimum_required(VERSION 2.8.3)
project(ground_estimator)

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
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

add_message_files(
  FILES
  ground_parameters.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

add_executable( ground_estimator
	src/ground_estimator.cpp
)

target_link_libraries( ground_estimator
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)