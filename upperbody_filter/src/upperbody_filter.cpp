#include <time.h>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>
#include <upperbody_detector/upperbody.h>
#include <upperbody_detector/upperbodyArray.h>

#include <Pixel.h>

/*boost::shared_ptr<cv::Mat> image(new cv::Mat);
boost::mutex image_mutex;
bool new_image_available_flag = false;

UpperBodyDetector* upperBodyDetector_ptr;*/
ros::Publisher humanPublisher;

/*void image_cb_ (const sensor_msgs::ImageConstPtr& callback_image) {
  image_mutex.lock ();

  *image = cv_bridge::toCvShare(callback_image, "bgr8")->image;
  new_image_available_flag = true;

  image_mutex.unlock ();
}*/
/*
void humans_cb_ (const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  ground_based_detector::humanArray potential_humans = *callback_humans;
  if (new_image_available_flag) {
    ground_based_detector::humanArray published_humans;
    
    new_image_available_flag = false;

    for(std::vector<ground_based_detector::human>::iterator it = potential_humans.humans.begin(); it != potential_humans.humans.end(); ++it) {
      if(upperBodyDetector_ptr->isHuman(*it)) published_humans.humans.push_back(*it);
    }
    
    humanPublisher.publish(published_humans);
  }
}*/
boost::mutex humans_mutex;
ground_based_detector::humanArray humans;

void humans_cb_ (const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  humans_mutex.lock();
  humans = *callback_humans;
  humans_mutex.unlock();
}

void rects_cb_ (const upperbody_detector::upperbodyArray::ConstPtr& callback_rects) {
  humans_mutex.lock();
  ground_based_detector::humanArray pclHumans = humans;
  humans_mutex.unlock();
  ground_based_detector::humanArray publishedHumans;
  
  struct timespec timer1, timer2;
  clock_gettime(0, &timer1);
  for(std::vector<ground_based_detector::human>::iterator person = pclHumans.humans.begin(); person != pclHumans.humans.end(); ++person) {
    Pixel personHead(Eigen::Vector3f(person->ttop_x, person->ttop_y, person->ttop_z) + Eigen::Vector3f(0,0.2,0));
    for(std::vector<upperbody_detector::upperbody>::const_iterator rect = callback_rects->upperbodies.begin(); rect != callback_rects->upperbodies.end(); ++rect) {
      cv::Rect upperbodyRect(rect->x, rect->y, rect->width, rect->height);
      if(personHead.in(upperbodyRect)) {
        publishedHumans.humans.push_back(*person);
        break;
      }
    }
  }
  clock_gettime(0, &timer2);
  ROS_ERROR_STREAM(timer2.tv_sec - timer1.tv_sec << " seconds, " << timer2.tv_nsec - timer1.tv_nsec << " nanoseconds.");

  humanPublisher.publish(publishedHumans);
}

int main(int argc, char *argv[]) {
	// Initialize ROS
  ros::init (argc, argv, "upperbody_filter");
  ros::NodeHandle nodeHandle;
  humanPublisher = nodeHandle.advertise<ground_based_detector::humanArray>("upperbody_filter/detected_human_clusters", 20);

  //std::string trained_upper_body_classifier_xml_path;
  //nodeHandle.getParam("upperbody_filter/trained_ubc_xml_path", trained_upper_body_classifier_xml_path);

  //ros::Subscriber subscriberImage = nodeHandle.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, image_cb_);
  ros::Subscriber subscriberHumans = nodeHandle.subscribe<ground_based_detector::humanArray>("ground_based_detector/detected_human_clusters", 1, humans_cb_);
  ros::Subscriber subscriberUpperbodies = nodeHandle.subscribe<upperbody_detector::upperbodyArray>("upperbody_detector/upperbodies", 10, rects_cb_);

  //UpperBodyDetector upperBodyDetector(image, trained_upper_body_classifier_xml_path, image_mutex);
  //upperBodyDetector_ptr = &upperBodyDetector;

  //while(!new_image_available_flag) 
  //  ros::spinOnce();



  ros::spin();

	return 0;
}