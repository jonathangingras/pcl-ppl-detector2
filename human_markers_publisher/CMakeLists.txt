cmake_minimum_required(VERSION 2.8.3)
project(human_markers_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  std_msgs
  tf
  interactive_markers
  ground_based_detector
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable( human_markers_publisher
	src/human_markers_publisher.cpp
)

target_link_libraries( human_markers_publisher
	${catkin_LIBRARIES}
)