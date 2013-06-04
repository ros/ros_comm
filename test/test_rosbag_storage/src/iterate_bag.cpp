#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "boost/foreach.hpp"

#include "ros/time.h"

int main(void)
{

  ros::Time::init();

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
      if(s->data == std::string("foo")) { 
        printf("Successfully checked string foo\n"); 
      }
      else {
        printf("Failed checked string foo\n"); 
      }

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
      if (i->data == 42) { 
        printf("Successfully checked value 42\n");
      }
      else {
        printf("Failed checked value 42.\n"); 
      }
;
  }

  bag.close();
  return 0;
}
