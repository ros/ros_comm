/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Open Robotics
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

/* Author: Martin Pecka */

/*
 * Test ros::init_options::NoSimTime.
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/topic_manager.h"

TEST(NoSimTime, isTimeValid)
{
  // We should be using the system time, so time should be valid right away
  ASSERT_TRUE(ros::Time::isValid());

  // Check that the use_sim_time parameter is set to true
  bool use_sim_time = false;
  ros::param::param("/use_sim_time", use_sim_time, use_sim_time);
  EXPECT_TRUE(use_sim_time);
  EXPECT_TRUE(ros::Time::isSystemTime());

  EXPECT_EQ(0u, ros::TopicManager::instance()->getNumSubscribers("/clock"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "sim_time_test", ros::init_options::NoSimTime);
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

