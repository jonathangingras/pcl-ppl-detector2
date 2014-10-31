#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include <ground_based_detector/humanArray.h>
#include <ground_based_detector/human.h>
#include <RegionNormalizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef typename pcl::PointCloud<PointT>::Ptr CloudTPtr;
typedef typename pcl::PointCloud<PointT>::ConstPtr CloudTConstPtr;

CloudTPtr cloud(new CloudT);
boost::mutex cloud_mutex;
bool new_cloud = false;
ros::Publisher* posesPublisher_ptr;

void cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& callback_cloud) {
	cloud_mutex.lock();
	pcl::fromROSMsg(*callback_cloud, *cloud);
	new_cloud = true;
	cloud_mutex.unlock();
}

inline double getROSYaw(const pcl::Normal& vector) {
  double angle = atan2(vector.normal_x, vector.normal_z);
  return isnan(angle) ? 0 : -angle;
}

void human_cb_ (const ground_based_detector::humanArrayConstPtr& humans) {
	if(!new_cloud) return;
	geometry_msgs::PoseArray poses;
	poses.header.frame_id = "/camera_depth_frame";
	
	unsigned int k = 0;
	for(std::vector<ground_based_detector::human>::const_iterator human = humans->humans.begin(); human != humans->humans.end(); ++human) {
		geometry_msgs::Pose pose;
		pose.position.x = human->ttop_z;
		pose.position.y = -human->ttop_x;
		pose.position.z = -human->ttop_y - 0.4;

		RegionNormalizer<PointT> normalizer(k++, Eigen::Vector3f(human->ttop_x, human->ttop_y + 0.4, human->ttop_z), cloud);

		cloud_mutex.lock();
		//pcl::Normal normal = normalizer.computeAverageNormal();
		double angle = normalizer.computeHistogramPeekAngle(10);
		cloud_mutex.unlock();

		pose.orientation = tf::createQuaternionMsgFromYaw(/*getROSYaw(normal)*/angle );
		poses.poses.push_back(pose);
	}

	new_cloud = false;
	posesPublisher_ptr->publish(poses);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "region_normalizer");
	ros::NodeHandle nodeHandle;

	ros::Publisher posesPublisher = nodeHandle.advertise<geometry_msgs::PoseArray>("region_normalizer/human_poses", 20);
	posesPublisher_ptr = &posesPublisher;
	ros::Subscriber cloudSubscriber = nodeHandle.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloud_cb_);
	ros::Subscriber humanSubscriber = nodeHandle.subscribe<ground_based_detector::humanArray>("upperbody_filter/detected_human_clusters", 1, human_cb_);

	ros::spin();
}