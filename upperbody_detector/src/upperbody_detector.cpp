#include <boost/thread.hpp>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <upperbody_detector/upperbody.h>
#include <upperbody_detector/upperbodyArray.h>

cv::CascadeClassifier classifier;
ros::Publisher upperbodyPublisher;

void image_cb_ (const sensor_msgs::ImageConstPtr& callback_image) {
	std::vector<cv::Rect> results;
	cv::Mat image = cv_bridge::toCvCopy(callback_image, "bgr8")->image;
	
  struct timespec timer1, timer2;
  clock_gettime(0, &timer1);
	classifier.detectMultiScale(image, results, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE );
  clock_gettime(0, &timer2);
  ROS_ERROR_STREAM(timer2.tv_sec - timer1.tv_sec << " seconds, " << timer2.tv_nsec - timer1.tv_nsec << " nanoseconds.");
	
	upperbody_detector::upperbodyArray publishedArray;
	for(std::vector<cv::Rect>::iterator it = results.begin(); it != results.end(); ++it) {
		upperbody_detector::upperbody publishedUBody;

		publishedUBody.x = it->x;
		publishedUBody.y = it->y;
		publishedUBody.height = it->height;
		publishedUBody.width = it->width;

		publishedArray.upperbodies.push_back(publishedUBody);
	}

	//cv::imshow("upperbody_detector", image);
	//cv::waitKey(30);

	upperbodyPublisher.publish(publishedArray);
}

int main(int argc, char** argv) {
	// Initialize ROS
  ros::init (argc, argv, "ground_based_detector");
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriberImage = nodeHandle.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, image_cb_);
	std::string trained_upper_body_classifier_xml_path;
  nodeHandle.getParam("upperbody_detector/trained_ubc_xml_path", trained_upper_body_classifier_xml_path);
	upperbodyPublisher = nodeHandle.advertise<upperbody_detector::upperbodyArray>("upperbody_detector/upperbodies", 20);

	classifier.load(trained_upper_body_classifier_xml_path);
	//cv::namedWindow("upperbody_detector");

	ros::spin();
}