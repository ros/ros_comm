// Test subscriber
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/publisher.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <std_msgs/Int8MultiArray.h>

#include <stdio.h>

void handleMsg(const std_msgs::Int8MultiArray& msg)
{
  static uint8_t previous = 0;
  uint8_t current = msg.data[0];
  uint8_t diff = current - previous;
  if (diff != 1)
  {
    ROS_WARN("Missing %u message(s)", diff - 1);
  }
  ROS_INFO("Got message %u", current);
  previous = msg.data[0];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("data", 1, &handleMsg, ros::TransportHints().udp());
  ros::spin();
  return 0;
}
