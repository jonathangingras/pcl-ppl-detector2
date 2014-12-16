#include <signal.h>
#include <time.h>

#include <fstream>
#include <sstream>
  
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <signal.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ground_estimator/ground_parameters.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Algorithm parameters:
double groud_filter_limit_up;
double groud_filter_limit_down;

ros::Publisher groundPlanePublisher;
PointCloudT::Ptr cloud(new PointCloudT);

boost::mutex mutex;

ros::Publisher markersPublisher;
std::string fixed_frame;

void publish_arrow (const ground_estimator::ground_parameters& parameters) {
  visualization_msgs::Marker m;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = fixed_frame;
  m.ns = "GROUND_NORMAL";
  m.id = 1;
  m.type = visualization_msgs::Marker::ARROW;
  m.pose.position.x = 3;
  m.pose.position.y = 0;
  m.pose.position.z = 0;
  m.scale.x = 2;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.a = 1;
  m.lifetime = ros::Duration(0.2);
  m.color.r = 255.0;
  m.color.g = 0.0;
  m.color.b = 0.0;

  m.pose.orientation.x = parameters.a/parameters.d;
  m.pose.orientation.y = parameters.b/parameters.d;
  m.pose.orientation.z = parameters.c/parameters.d;
  m.pose.orientation.w = 1;

  markersPublisher.publish(m);
}

void cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& callback_cloud) {
  PointCloudT::ConstPtr const_cloud(cloud);
  PointCloudT::Ptr lowerHeigthCloud(new PointCloudT);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (const_cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (groud_filter_limit_up, groud_filter_limit_down); // lower heigth axis(Y) (Y points to ground i.e. center is 0, ceil is negative, and floor is positive)

  mutex.lock();
  
  pcl::fromROSMsg(*callback_cloud, *cloud);
  // (GPE) Ground plane estimation
  // (GPE) A. Get lower heigth cloud
  pass.filter (*lowerHeigthCloud);

  mutex.unlock();

  // (GPE) B. Setup RANSAC on the lower height cloud
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> ground_finder;
  Eigen::Vector3f depth_axis(0.0,0.0,1.0); // depth axis Z
  ground_finder.setOptimizeCoefficients(true);
  ground_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  ground_finder.setMethodType(pcl::SAC_RANSAC);
  ground_finder.setDistanceThreshold(0.015);
  ground_finder.setAxis(depth_axis);
  ground_finder.setInputCloud(lowerHeigthCloud);

  // (GPE) C. Segment and find ground plane coefficients
  pcl::ModelCoefficients ground_coefficients;
  try{ 
  	ground_finder.segment(*ground_indices, ground_coefficients);
  } catch (...) {
  	ROS_ERROR("error segmenting the ground plane!");
  	return;
  }

  // (GPE) D. Convert pcl::ModelCoefficients to Eigen::VectorXf (for people_detector compatibility)
  ground_estimator::ground_parameters published_plane;
  published_plane.a = ground_coefficients.values[0];
  published_plane.b = ground_coefficients.values[1];
  published_plane.c = ground_coefficients.values[2];
  published_plane.d = ground_coefficients.values[3];

  groundPlanePublisher.publish(published_plane);
  publish_arrow(published_plane);
}


int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "ground_estimator");
  ros::NodeHandle nodeHandle;

  // Read parameters
  nodeHandle.getParam("ground_estimator/fixed_frame", fixed_frame);
  nodeHandle.getParam("ground_estimator/grf_up", groud_filter_limit_up);
  nodeHandle.getParam("ground_estimator/grf_down", groud_filter_limit_down);

  // Read Kinect live stream:
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subscriberCloud = nodeHandle.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb_);
  groundPlanePublisher = nodeHandle.advertise<ground_estimator::ground_parameters>("ground_estimator/ground_plane_parameters", 20);
  markersPublisher = nodeHandle.advertise<visualization_msgs::Marker>("ground_estimator/plane_parameters", 20);
  // Main loop:
  ros::spin();

  return 0;
}

