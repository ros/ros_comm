#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/Int32.h"

#include "boost/foreach.hpp"
#include <gtest/gtest.h>

void writeBags(rosbag::CompressionType a, rosbag::CompressionType b) {
    using std::swap;
    rosbag::Bag bag1("/tmp/swap1.bag", rosbag::bagmode::Write);
    rosbag::Bag bag2("/tmp/swap2.bag", rosbag::bagmode::Write);

    // In the end "/tmp/swap1.bag" should have CompressionType a and contain two messages of value a.
    // "/tmp/swap2.bag" should have CompressionType b and contain two messages of value b.
    // We use these pointers to track the bags accordingly.

    rosbag::Bag* a_bag = &bag1;
    rosbag::Bag* b_bag = &bag2;

    std_msgs::Int32 a_msg, b_msg;
    a_msg.data = a;
    b_msg.data = b;

    swap(bag1, bag2);
    swap(a_bag, b_bag);

    a_bag->setCompression(a);
    b_bag->setCompression(b);

    swap(bag1, bag2);
    swap(a_bag, b_bag);

    a_bag->write("/data", ros::Time::now(), a_msg);
    b_bag->write("/data", ros::Time::now(), b_msg);

    swap(bag1, bag2);
    swap(a_bag, b_bag);

    a_bag->write("/data", ros::Time::now(), a_msg);
    b_bag->write("/data", ros::Time::now(), b_msg);

    swap(bag1, bag2);

    bag1.close();
    bag2.close();

    swap(bag1, bag2);
}

void readBags(rosbag::CompressionType a, rosbag::CompressionType b) {
    using std::swap;
    rosbag::Bag bag1("/tmp/swap1.bag", rosbag::bagmode::Read);
    rosbag::Bag bag2("/tmp/swap2.bag", rosbag::bagmode::Read);

    rosbag::Bag* a_bag = &bag1;
    rosbag::Bag* b_bag = &bag2;

    swap(bag1, bag2);
    swap(a_bag, b_bag);

    // only valid when writing
    //EXPECT_EQ(a_bag->getCompression(), a);
    //EXPECT_EQ(b_bag->getCompression(), b);

    std::vector<std::string> topics;
    topics.push_back("data");

    rosbag::View a_view(*a_bag, rosbag::TopicQuery(topics));
    rosbag::View b_view(*b_bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, a_view)
    {
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        ASSERT_TRUE(i);
        EXPECT_EQ(i->data, a);
    }
    BOOST_FOREACH(rosbag::MessageInstance const m, b_view)
    {
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        ASSERT_TRUE(i);
        EXPECT_EQ(i->data, b);
    }
}

TEST(rosbag_storage, swap_bags)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            writeBags(rosbag::CompressionType(i), rosbag::CompressionType(j));
            readBags(rosbag::CompressionType(i), rosbag::CompressionType(j));
        }
    }
}

int main(int argc, char **argv) {
    ros::Time::init();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
