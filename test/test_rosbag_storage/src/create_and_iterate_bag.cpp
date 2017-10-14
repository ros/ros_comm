#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

#include "boost/foreach.hpp"
#include <gtest/gtest.h>

template<typename T>
T make_std_msg(const typename T::_data_type& data) {
  T msg;
  msg.data = data;
  return msg;
}

void create_test_bag(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);

  bag.write("chatter", ros::Time::now(), make_std_msg<std_msgs::String>("foo"));
  bag.write("numbers", ros::Time::now(), make_std_msg<std_msgs::Int32>(42));

  bag.close();
}

void create_a_different_test_bag(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);

  bag.write("a", ros::Time::now(), make_std_msg<std_msgs::String>("Hello World!"));
  bag.write("b", ros::Time::now(), make_std_msg<std_msgs::Int8>(1));
  bag.write("c", ros::Time::now(), make_std_msg<std_msgs::UInt16>(2));
  bag.write("d", ros::Time::now(), make_std_msg<std_msgs::Int32>(3));
  bag.write("e", ros::Time::now(), make_std_msg<std_msgs::UInt64>(4));

  bag.close();
}

const char* bag_filename = "/tmp/rosbag_storage_create_and_iterate_bag.bag";
const char* bag_filename2 = "/tmp/rosbag_storage_create_and_iterate_bag2.bag";

TEST(rosbag_storage, iterator_copy_constructor)
{
  // copy ctor
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("numbers"));
  rosbag::View::const_iterator it0 = view.begin();
  EXPECT_EQ(42, it0->instantiate<std_msgs::Int32>()->data);
  rosbag::View::const_iterator it1(it0);
  EXPECT_EQ(it0, it1);
  EXPECT_EQ(42, it1->instantiate<std_msgs::Int32>()->data);
  ++it1;
  EXPECT_NE(it0, it1);
  EXPECT_EQ(42, it0->instantiate<std_msgs::Int32>()->data);
}

TEST(rosbag_storage, iterator_copy_assignment)
{
  // copy assignment
  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("numbers"));
  rosbag::View::const_iterator it0 = view.begin();
  EXPECT_EQ(42, it0->instantiate<std_msgs::Int32>()->data);
  rosbag::View::const_iterator it1;
  it1 = it0;
  EXPECT_EQ(it0, it1);
  EXPECT_EQ(42, it1->instantiate<std_msgs::Int32>()->data);
  ++it1;
  EXPECT_NE(it0, it1);
  EXPECT_EQ(42, it0->instantiate<std_msgs::Int32>()->data);
}

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

void serialize_bag(rosbag::Bag& bag, const char* filename)
{
  bag.open(filename);

  rosbag::View bag_view(bag);
  std::vector<uint8_t> buffer;

  BOOST_FOREACH(const rosbag::MessageInstance& msg_instance, bag_view)
  {
    const size_t msg_size  = msg_instance.size();
    buffer.resize(msg_size);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg_instance.write(stream);

    printf("reading: %s\n",  msg_instance.getTopic().c_str() );
  }
  bag.close();
}

TEST(rosbag_storage, reuse_bag)
{
  rosbag::Bag bag;

  serialize_bag(bag, bag_filename);
  serialize_bag(bag, bag_filename2);
}

int main(int argc, char **argv) {
    ros::Time::init();
    create_test_bag(bag_filename);
    create_a_different_test_bag(bag_filename2);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
