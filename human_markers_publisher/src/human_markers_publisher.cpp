#include <signal.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>

//ground_based_detector::humanArray potential_humans;
ros::Publisher markersPublisher;
std::string fixed_frame;

void humans_cb_ (const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  unsigned int k = 0;
  visualization_msgs::MarkerArray marker_array;
  for(std::vector<ground_based_detector::human>::const_iterator it = callback_humans->humans.begin(); it != callback_humans->humans.end(); ++it) {
  	visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = fixed_frame;
    m.ns = "HUMANS";
    m.id = ++k;
    m.type = visualization_msgs::Marker::CYLINDER;//m.CYLINDER;
    m.pose.position.x = it->tcenter_z;
    m.pose.position.y = -it->tcenter_x;
    m.pose.position.z = -it->tcenter_y;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = it->height;
    m.color.a = 0.5;
    m.lifetime = ros::Duration(0.2);
    m.color.r = 0;
    m.color.g = 255.0;
    m.color.b = 0;
  	//markersPublisher.publish(m);
    marker_array.markers.push_back(m);
  }
  markersPublisher.publish(marker_array);
}

bool interrupted = false;
void signal_handler(int signal) {
    interrupted = true;
}

int main(int argc, char *argv[]) {
	// Initialize ROS
  ros::init (argc, argv, "upperbody_filter");
  ros::NodeHandle nodeHandle;
  //tf::TransformListener transform_listener;

  nodeHandle.getParam("human_markers_publisher/fixed_frame", fixed_frame);
  ros::Subscriber subscriberHumans = nodeHandle.subscribe<ground_based_detector::humanArray>("upperbody_filter/detected_human_clusters", 1, humans_cb_);
  markersPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("human_markers_publisher/human_markers", 20);

  signal(SIGINT, signal_handler);
  while(ros::ok() && !interrupted) {
  	ros::spinOnce();
  }

	return 0;
}