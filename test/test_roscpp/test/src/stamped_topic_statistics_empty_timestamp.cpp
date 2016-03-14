/*
 * Copyright (c) 2015, Eric Perko
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <test_roscpp/TestWithHeader.h>

void callback(const test_roscpp::TestWithHeaderConstPtr&)
{
  // No operation needed here
}

TEST(TopicStatistics, empty_timestamp_crash_check)
{
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<test_roscpp::TestWithHeader>("test_with_empty_timestamp", 0);
  ros::Subscriber sub = nh.subscribe("test_with_empty_timestamp", 0, callback);

  ros::Time start = ros::Time::now();
  ros::Duration time_to_publish(10.0);
  while ( (ros::Time::now() - start) < time_to_publish )
  {
    test_roscpp::TestWithHeader msg;
    msg.header.frame_id = "foo";
    // Don't fill in timestamp so that it defaults to 0.0

    pub.publish(msg);
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  SUCCEED();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "stamped_topic_statistics_empty_timestamp");

  return RUN_ALL_TESTS();
}