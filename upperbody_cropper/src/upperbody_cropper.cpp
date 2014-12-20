/*
 * http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php
 * http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/ua_vision/background_filters/src/ground_filter.cpp
 * http://pointclouds.org/documentation/tutorials/passthrough.php
 */
#include <signal.h>
#include <time.h>

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/crop_box.h>

#include <boost/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex, cloud_out_mutex;
bool new_cloud_available_flag = false;

bool new_dpm = false, new_vj = false;
PointCloudT::Ptr cloud(new PointCloudT), cloud_out(new PointCloudT);
sensor_msgs::PointCloud2::Ptr ros_cloud_out(new sensor_msgs::PointCloud2);
ros::Publisher cleanedCloudPublisher;

bool interrupted = false;
void signal_handler(int signal) {
    interrupted = true;
}

void cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& callback_cloud) {
  cloud_mutex.lock ();

  pcl::fromROSMsg(*callback_cloud, *cloud);
  new_cloud_available_flag = true;

  cloud_mutex.unlock ();
}

void addTorsoToCloud(PointCloudT& _cloud_out, PointCloudT::Ptr& _cloud_in, const ground_based_detector::human& _human) {
  float cube_edge = _human.height / 2;//_human.tcenter_y - _human.ttop_y - 0.3f;

  //Define your cube with two points in space:
  Eigen::Vector4f minPoint;
  minPoint[0] = 0;  // define minimum point x
  minPoint[1] = 0;  // define minimum point y
  minPoint[2] = 0;  // define minimum point z
  Eigen::Vector4f maxPoint;
  minPoint[0] = cube_edge;  // define max point x
  minPoint[1] = cube_edge;  // define max point y
  minPoint[2] = cube_edge;  // define max point z

  //Define translation and rotation ( this is optional)
  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = _human.ttop_x;// + (0.5 * cube_edge);  
  boxTranslatation[1] = _human.ttop_y + 0.3f;  
  boxTranslatation[2] = _human.ttop_z;// - (0.5 * cube_edge);

  pcl::CropBox<PointT> cropFilter;
  cropFilter.setInputCloud (_cloud_in);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);

  cloud_out_mutex.lock();
  cropFilter.filter (_cloud_out);
  cloud_out_mutex.unlock();
}

void human_cb_(const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  PointCloudT::Ptr cloud_in(new PointCloudT);
  cloud_mutex.lock();
  *cloud_in = *cloud;
  cloud_mutex.unlock();

  for(std::vector<ground_based_detector::human>::const_iterator human = callback_humans->humans.begin(); human != callback_humans->humans.end(); ++human) {
    addTorsoToCloud(*cloud_out, cloud_in, *human);
  }
}

void human_cb_dpm_(const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  human_cb_(callback_humans);
  new_dpm = true;
}

void human_cb_vj_(const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  human_cb_(callback_humans);
  new_vj = true;
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "upperbody_cropper");
  ros::NodeHandle nodeHandle;
  cleanedCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("upperbody_cropper/torso_cloud", 20);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subscriberCloud = nodeHandle.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 20, cloud_cb_);
  ros::Subscriber subscriberHumansVJ = nodeHandle.subscribe<ground_based_detector::humanArray>("/upperbody_filter/detected_human_clusters", 10, human_cb_vj_);
  ros::Subscriber subscriberHumansDPM = nodeHandle.subscribe<ground_based_detector::humanArray>("/upperbody_dpm_filter/detected_human_clusters", 10, human_cb_dpm_);
  
  ros_cloud_out->header.frame_id = "/camera_depth_frame";

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    ros::spinOnce();
  
  new_cloud_available_flag = false;

  while(ros::ok()) {
    ros::spinOnce();
    if(new_dpm && new_vj) {
      new_dpm = false;
      new_vj = false;
      cloud_out_mutex.lock();
      pcl::toROSMsg(*cloud_out, *ros_cloud_out);
      cloud_out_mutex.unlock();
      cleanedCloudPublisher.publish(ros_cloud_out);
    }
  }

  return 0;
}

