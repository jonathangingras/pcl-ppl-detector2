#include <time.h>

#include <boost/thread.hpp>
#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>

#include <UpperBodyDPMFilter.h>

UpperBodyDPMFilter::CandidateArray::Ptr dpmCandidates(new UpperBodyDPMFilter::CandidateArray);
boost::mutex dpm_mutex;
bool new_dpm_available_flag = false;

UpperBodyDPMFilter* UpperBodyDPMFilter_ptr;
ros::Publisher humanPublisher;


void dpm_cb_ (const UpperBodyDPMFilter::CandidateArray::ConstPtr& callback_candidates) {
  dpm_mutex.lock ();

  *dpmCandidates = *callback_candidates;
  new_dpm_available_flag = true;

  dpm_mutex.unlock ();
}

void humans_cb_ (const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  ground_based_detector::humanArray potential_humans = *callback_humans;
  if (new_dpm_available_flag) {
    new_dpm_available_flag = false;

    ground_based_detector::humanArray published_humans;

    for(std::vector<ground_based_detector::human>::iterator person = potential_humans.humans.begin(); person != potential_humans.humans.end(); ++person) {
      if(UpperBodyDPMFilter_ptr->humanMatchesDPMParts(*person, *dpmCandidates)) {
        published_humans.humans.push_back(*person);
      }
    }
    
    humanPublisher.publish(published_humans);
  }
}

int main(int argc, char *argv[]) {
	// Initialize ROS
  ros::init (argc, argv, "upperbody_filter");
  ros::NodeHandle nodeHandle;
  humanPublisher = nodeHandle.advertise<ground_based_detector::humanArray>("/upperbody_dpm_filter/detected_human_clusters", 20);

  UpperBodyDPMFilter UpperBodyDPMFilter(dpm_mutex);
  UpperBodyDPMFilter_ptr = &UpperBodyDPMFilter;

  ros::Subscriber DPMsubscriber = nodeHandle.subscribe<object_recognition_by_parts::CandidateArray>("/pbd/Person_8parts/candidates_by_part_centers", 1, dpm_cb_);
  ros::Subscriber PCLsubscriber = nodeHandle.subscribe<ground_based_detector::humanArray>("ground_based_detector/detected_human_clusters", 1, humans_cb_);

  while(!new_dpm_available_flag) 
    ros::spinOnce();

  ros::spin();

	return 0;
}