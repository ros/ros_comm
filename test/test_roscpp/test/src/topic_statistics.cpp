/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <boost/shared_ptr.hpp>

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/statistics.h>
#include <ros/subscription_callback_helper.h>
#include <ros/time.h>
#include <std_msgs/String.h>

/**
 * \brief Simulate receiving 10000 subscription callbacks of std_msgs/String messages with
 *        increasing timestamp, starting from time 1.0.
 * \param[in] logger The statistics logger to call when a simulated message is received.
 */
void lotsOfMessages(ros::StatisticsLogger& logger)
{
  ros::Time now(1, 0);
  ros::Duration dt(0.1);
  for (size_t i = 0; i < 10000; ++i)
  {
    ros::Time::setNow(now);
    logger.callback(nullptr, "topic", "caller", {}, 100, now, false, 0);
    now += dt;
  }
}

/**
 * \brief Unused.
 */
void fooCb(const std_msgs::String&)
{
}

/**
 * \brief Test that resetting ROS time while a lot of messages is published doesn't
 *        congest the statistics (https://github.com/ros/ros_comm/issues/2249).
 */
TEST(TopicStatistics, TimeJumps)
{
  ros::StatisticsLogger logger;
  auto h = boost::make_shared<ros::SubscriptionCallbackHelperT<std_msgs::String>>(&fooCb);
  logger.init(h);
  
  // Make 4 repetitions of receiving 10000 messages. First repetition is okay. If the
  // bug from doc-comment is not fixed, the following iterations will take considerably
  // longer because the internal lists are never cleared.
  
  ros::WallDuration initialDuration;
  for (size_t i = 0; i < 4; ++i)
  {
    auto start = ros::WallTime::now();
    lotsOfMessages(logger);
    auto duration = ros::WallTime::now() - start;
    if (i == 0)
      initialDuration = duration;
    else
      EXPECT_GT(initialDuration * 5, duration);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "topic_statistics");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
