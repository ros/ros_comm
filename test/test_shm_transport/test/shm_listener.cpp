#include "ros/ros.h"
#include "ros/shm_topic.hpp"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr & msg) {
  char str[21] = {'\0'};
  strncpy(str, msg->data.c_str(), 20);
  ROS_INFO("I heard: [%s]", str);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "shm_listener", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  shm_transport::Topic t(n);
  shm_transport::Subscriber< std_msgs::String > s = t.subscribe("shm_test_topic", 60, chatterCallback);
  ros::spin();
  return 0;
}
