#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "ros/time.h"

int main(void)
{

  ros::Time::init();

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);
  
  std_msgs::String str;
  str.data = std::string("foo");
  
  std_msgs::Int32 i;
  i.data = 42;
  
  bag.write("chatter", ros::Time::now(), str);
  bag.write("numbers", ros::Time::now(), i);
  
  bag.close();
  

  return 0;
}
