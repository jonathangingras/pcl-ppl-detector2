cmake_minimum_required(VERSION 2.8.3)
project(region_normalizer)

find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  ground_based_detector
)

find_package( PCL REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

catkin_package()

add_executable( region_normalizer
  src/region_normalizer.cpp
)

target_link_libraries( region_normalizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)