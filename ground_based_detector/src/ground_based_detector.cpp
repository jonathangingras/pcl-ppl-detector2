/*
 * http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php
 * http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/ua_vision/background_filters/src/ground_filter.cpp
 * http://pointclouds.org/documentation/tutorials/passthrough.php
 */
#include <signal.h>
#include <time.h>

#include <fstream>
#include <sstream>

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;
bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud(new PointCloudT);

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

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "ground_based_detector");
  ros::NodeHandle nodeHandle;
  ros::Publisher humanPublisher = nodeHandle.advertise<ground_based_detector::humanArray>("ground_based_detector/detected_human_clusters", 20);
  ros::Publisher groundPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("ground_based_detector/ground_cloud", 1);

  // Algorithm parameters:
  std::string svm_filename;
  std::string trained_upper_body_classifier_xml_path;
  double min_confidence;
  double min_height;
  double max_height;
  double voxel_size;
  double groud_filter_limit_up;
  double groud_filter_limit_down;
  std::string results_dir;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Read parameters
  nodeHandle.getParam("ground_based_detector/svm", svm_filename);
  nodeHandle.getParam("ground_based_detector/min_confidence", min_confidence);
  nodeHandle.getParam("ground_based_detector/min_h", min_height);
  nodeHandle.getParam("ground_based_detector/max_h", max_height);
  nodeHandle.getParam("ground_based_detector/voxel_size", voxel_size);
  nodeHandle.getParam("ground_based_detector/grf_up", groud_filter_limit_up);
  nodeHandle.getParam("ground_based_detector/grf_down", groud_filter_limit_down);
  nodeHandle.getParam("ground_based_detector/results_dir", results_dir);

  // Read Kinect live stream:
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber subscriberCloud = nodeHandle.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb_);
  
  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    ros::spinOnce();
  
  new_cloud_available_flag = false;

  // (GPE) Ground plane estimation:

  // (GPE) A. Get lower heigth cloud
  PointCloudT::ConstPtr const_cloud(cloud);
  PointCloudT::Ptr lowerHeigthCloud(new PointCloudT);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (const_cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (groud_filter_limit_up, groud_filter_limit_down); // lower heigth axis(Y) (Y points to ground i.e. center is 0, ceil is negative, and floor is positive)
  pass.filter (*lowerHeigthCloud);

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
  ground_finder.segment(*ground_indices, ground_coefficients);

  // (GPE) D. Convert pcl::ModelCoefficients to Eigen::VectorXf (for people_detector compatibility)
  Eigen::VectorXf plane_parameters;
  plane_parameters.resize(4);
  plane_parameters << ground_coefficients.values[0], ground_coefficients.values[1], ground_coefficients.values[2], ground_coefficients.values[3];
  
  // Extract the ground cloud
  PointCloudT::ConstPtr const_lowerHeigthCloud(lowerHeigthCloud);
  PointCloudT::Ptr extractedGroundCloud(new PointCloudT);
  pcl::ExtractIndices<PointT> groundExtractor;
  groundExtractor.setInputCloud (const_lowerHeigthCloud);
  groundExtractor.setIndices(ground_indices);
  groundExtractor.setNegative(false);
  groundExtractor.filter(*extractedGroundCloud);

  sensor_msgs::PointCloud2 ground_cloud_msg;
  pcl::toROSMsg(*extractedGroundCloud, ground_cloud_msg);
  ground_cloud_msg.header.frame_id = "/camera_depth_frame";

  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);                 // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
  //people_detector.setSensorPortraitOrientation(true);            // set sensor orientation to vertical

  people_detector.setInputCloud(cloud);

  // Main loop:
  signal(SIGINT, signal_handler);
  while (ros::ok() && !interrupted) {
    groundPublisher.publish(ground_cloud_msg);
    
    std::vector<pcl::people::PersonCluster<PointT> > clusters; // vector containing persons clusters

    people_detector.setGround(plane_parameters); // set floor coefficients

    if (new_cloud_available_flag && cloud_mutex.try_lock()) // if a new cloud is available
    {
      new_cloud_available_flag = false;

      // Perform people detection on the new cloud:
      people_detector.compute(clusters); // perform people detection

      cloud_mutex.unlock();
      ground_based_detector::humanArray published_clusters;

      // draw boxes
      unsigned int k = 0;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getPersonConfidence() > min_confidence) // draw only people with confidence above a threshold an ok from opencv upper body detector
        { 
          ground_based_detector::human published_human;
          published_human.height = it->getHeight();
          published_human.tcenter_x = it->getTCenter()[0];
          published_human.tcenter_y = it->getTCenter()[1];
          published_human.tcenter_z = it->getTCenter()[2];
          published_human.ttop_x = it->getTTop()[0];
          published_human.ttop_y = it->getTTop()[1];
          published_human.ttop_z = it->getTTop()[2];

          published_clusters.humans.push_back(published_human);
        }

      }
      humanPublisher.publish(published_clusters);
    }

    plane_parameters = people_detector.getGround();
    ros::spinOnce();
  }

  if(interrupted) std::cout << std::endl;
  return 0;
}

