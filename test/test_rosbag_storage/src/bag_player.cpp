#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/bag_player.h>
#include <std_msgs/UInt64.h>

#include <gtest/gtest.h>
#include <boost/bind.hpp>

/**
 * @brief Helper function to create std_msgs::* messages.
 * @param[in] data Message data/payload.
 * @return std_msgs::* message with the given data/payload.
 */
template <typename T>
T make_std_msg(const typename T::_data_type& data)
{
  T msg;
  msg.data = data;
  return msg;
}

/**
 * @brief BagPlayer test, which creates a test bag once and provides callbacks for rosbag::MessageInstance and
 * std_msgs::UInt64.
 */
class BagPlayerTest : public testing::Test
{
public:
  /**
   * @brief Constructor; simply resets the number of messages counted.
   */
  BagPlayerTest() : num_messages(0)
  {
  }

  /**
   * @brief Destructor.
   */
  ~BagPlayerTest()
  {
  }

  /**
   * @brief Callback for rosbag::MessageInstance.
   * @param[in] m rosbag::MessageInstance.
   */
  void callbackMessageInstance(const rosbag::MessageInstance& m)
  {
    callbackMessage(m.instantiate<std_msgs::UInt64>());
  }

  /**
   * @brief Callback for std_msgs::UInt64 message.
   * @param[in] msg Message.
   */
  void callbackMessage(const std_msgs::UInt64ConstPtr& msg)
  {
    EXPECT_EQ(num_messages, msg->data);

    ++num_messages;
  }

  /**
   * @brief Static test case setup (called only once), which create a test bag.
   */
  static void SetUpTestCase()
  {
    ros::Time::init();

    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Write);

    for (size_t i = 0; i < bag_messages; ++i)
    {
      bag.write(topic, ros::Time::now(), make_std_msg<std_msgs::UInt64>(i));
    }

    bag.close();
  }

  /// Test bag filename:
  static const std::string bag_filename;

  /// Test bag number of messages:
  static const size_t bag_messages;

  /// Test bag topic name:
  static const std::string topic;

  /// Number of messages counter:
  size_t num_messages;
};

// Initialize static attributes:
const std::string BagPlayerTest::bag_filename = "/tmp/bag_player.bag";
const size_t BagPlayerTest::bag_messages = 123;
const std::string BagPlayerTest::topic = "numbers";

/// Global number of messages counter (for bare functions):
size_t callback_num_messages(0);

/**
 * @brief Callback for std_msgs::UInt64 message.
 * @param[in] msg Message.
 */
void callback_message(const std_msgs::UInt64ConstPtr& msg)
{
  EXPECT_EQ(msg->data, callback_num_messages);

  ++callback_num_messages;
}

/**
 * @brief Callback for rosbag::MessageInstance.
 * @param[in] m rosbag::MessageInstance.
 */
void callback_message_instance(const rosbag::MessageInstance& m)
{
  callback_message(m.instantiate<std_msgs::UInt64>());
}

// Test bare function that takes ROS message (std_msgs::UInt64):
TEST_F(BagPlayerTest, bag_player_message)
{
  callback_num_messages = 0;

  rosbag::BagPlayer player(bag_filename);
  player.register_callback<std_msgs::UInt64>(topic, callback_message);
  player.start_play();

  EXPECT_EQ(bag_messages, callback_num_messages);
}

// Test bare function that takes rosbag::MessageInstance:
TEST_F(BagPlayerTest, bag_player_message_instance)
{
  callback_num_messages = 0;

  rosbag::BagPlayer player(bag_filename);
  player.register_callback<rosbag::MessageInstance>(topic, callback_message_instance);
  player.start_play();

  EXPECT_EQ(bag_messages, callback_num_messages);
}

// Test class method that takes ROS message (std_msgs::UInt64):
TEST_F(BagPlayerTest, bag_player_message_class)
{
  rosbag::BagPlayer player(bag_filename);
  player.register_callback<std_msgs::UInt64>(topic, boost::bind(&BagPlayerTest::callbackMessage, this, _1));
  player.start_play();

  EXPECT_EQ(bag_messages, num_messages);
}

// Test class method that takes rosbag::MessageInstance:
TEST_F(BagPlayerTest, bag_player_message_instance_class)
{
  rosbag::BagPlayer player(bag_filename);
  player.register_callback<rosbag::MessageInstance>(topic,
                                                    boost::bind(&BagPlayerTest::callbackMessageInstance, this, _1));
  player.start_play();

  EXPECT_EQ(bag_messages, num_messages);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
