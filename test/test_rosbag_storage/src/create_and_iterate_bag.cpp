#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

#include "boost/foreach.hpp"
#include <gtest/gtest.h>

void create_test_bag(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);

  std_msgs::String str;
  str.data = std::string("foo");

  std_msgs::Int32 i;
  i.data = 42;

  bag.write("chatter", ros::Time::now(), str);
  bag.write("numbers", ros::Time::now(), i);

  bag.close();
}

const char* bag_filename = "/tmp/rosbag_storage_create_and_iterate_bag.bag";

TEST(rosbag_storage, iterate_bag)
{
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
    {
      if(s->data == std::string("foo")) {
        printf("Successfully checked string foo\n");
      }
      else
      {
        printf("Failed checked string foo\n");
        FAIL();
      }
    }

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
    {
      if (i->data == 42) {
        printf("Successfully checked value 42\n");
      }
      else
      {
        printf("Failed checked value 42.\n");
        FAIL();
      }
    }
  }

  bag.close();
}

int main(int argc, char **argv) {
    ros::Time::init();
    create_test_bag(bag_filename);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
