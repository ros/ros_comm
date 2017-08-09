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

  ros::Duration delay_to_publish;
  try {
    delay_to_publish.fromSec(10.0);
  } catch (const std::runtime_error & e) {
    ROS_FATAL_STREAM("It was in the duration: " << e.what());
    FAIL();
  }
    
  ros::Time start = ros::Time::now();
  ros::Time time_to_publish;
  try {
    time_to_publish = start + delay_to_publish;
  } catch (const std::runtime_error & e) {
    ROS_FATAL_STREAM("It was in the addition: " << e.what() << start.toNSec() << " " << delay_to_publish.toSec());
    FAIL();
  }

  ROS_FATAL("Starting the loop");
  unsigned i = 0;
  try {
    for (; i < 1000; ++i) { //while ( ros::Time::now() < time_to_publish )
      try {
	try {
	  test_roscpp::TestWithHeader msg;
	} catch (const std::runtime_error & e) {
	  ROS_FATAL_STREAM("It is mad when you create a msg");
	  FAIL();
	}
	
	test_roscpp::TestWithHeader msg;
	try {
	  msg.header.frame_id = "foo";
	} catch (const std::runtime_error & e) {
	  ROS_FATAL_STREAM("It is mad when set the frame_id");
	  FAIL();
	}
	
	try {
	  pub.publish(msg);
	} catch (const std::runtime_error & e) {
	  ROS_FATAL_STREAM("It was in the publish: " << e.what());
	  FAIL();
	}
	try {
	  ros::spinOnce();
	} catch (const std::runtime_error & e) {
	  ROS_FATAL_STREAM("It was in the spin: " << e.what());
	  FAIL();
	}
	try {
	  //ros::WallDuration(0.01).sleep();
	} catch (const std::runtime_error & e) {
	  ROS_FATAL_STREAM("It was in the sleep: " << e.what());
	  FAIL();
	}
      } catch(const std::runtime_error & e) {
	ROS_FATAL_STREAM("Uncaught in the loop on iter " << i << " with: " << e.what());
	FAIL();
      } catch(...) {
	ROS_FATAL_STREAM("Uncaught in the loop on iter " << i);
	FAIL();
      }
    }
  } catch(const std::runtime_error & e) {
    ROS_FATAL_STREAM("Uncaught outside the loop on iter " << i << " with: " << e.what());
    FAIL();
  } catch(...) {
    ROS_FATAL_STREAM("Uncaught outside the loop on iter " << i);
    FAIL();
  }
  ROS_FATAL_STREAM("Done testing the message");

  SUCCEED();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "stamped_topic_statistics_empty_timestamp");

  return RUN_ALL_TESTS();
}
