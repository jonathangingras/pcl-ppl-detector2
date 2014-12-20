#include <signal.h>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <ground_based_detector/human.h>
#include <ground_based_detector/humanArray.h>
#include <JSONPointPrinter.hpp>

JSONPrinter* jsonPrinter;

void humans_cb_ (const ground_based_detector::humanArray::ConstPtr& callback_humans) {
  for(std::vector<ground_based_detector::human>::const_iterator it = callback_humans->humans.begin(); it != callback_humans->humans.end(); ++it) {
    jsonPrinter->addElement(it->ttop_x, it->ttop_z, 1);
  }
  jsonPrinter->printSample();
}

bool interrupted = false;
void signal_handler(int signal) {
    interrupted = true;
}

std::string resultFileName(std::string& provider) {
  time_t now = time(NULL);
  std::ostringstream os;
  os << "/tmp/" << now << "_humans_" << provider << ".json";
  return os.str();
}

int main(int argc, char *argv[]) {
	// Initialize ROS
  ros::init (argc, argv, "human_json_outputer");
  ros::NodeHandle nodeHandle;
  std::string filter_node, provider;

  nodeHandle.getParam("human_json_outputer/filter_node", filter_node);
  nodeHandle.getParam("human_json_outputer/provider", provider);

  std::ofstream ofile(resultFileName(provider).c_str());
  JSONPrinter _jsonPrinter("legs", ofile);
  jsonPrinter = &_jsonPrinter;

  ros::Subscriber subscriberHumans = nodeHandle.subscribe<ground_based_detector::humanArray>(filter_node, 20, humans_cb_);
  
  signal(SIGINT, signal_handler);
  while(ros::ok() && !interrupted) {
  	ros::spinOnce();
  }

	return 0;
}