cmake_minimum_required(VERSION 2.8.3)
project(upperbody_filter)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  ground_based_detector
  upperbody_detector
)

find_package( PCL REQUIRED )
#find_package( OpenCV REQUIRED )

include_directories(
	${catkin_INCLUDE_DIRS}
	#${OpenCV_INCLUDE_DIRS}
	include
)

link_directories(
  /usr/local/lib
)

catkin_package()

add_executable( upperbody_filter
	#src/upperbody_filter.cpp
  src/upperbody_filter_original.cpp
)

target_link_libraries( upperbody_filter
	${catkin_LIBRARIES}
	#${OpenCV_LIBRARIES}
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