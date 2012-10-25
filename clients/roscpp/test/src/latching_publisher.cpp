/*
 * Copyright (c) 2NULLNULL8, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Josh Faust */

#include <string>
#include <sstream>
#include <fstream>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <roscpp/TestArray.h>

using namespace ros;
using namespace roscpp;

std::string g_node_name = "test_latching_publisher";

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const TestArray::ConstPtr& msg)
  {
    ++count_;
    last_msg_ = msg;
  }

  int32_t count_;
  TestArray::ConstPtr last_msg_;
};

TEST(RoscppLatchingPublisher, nonLatching)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<TestArray>("test", 1, false);
  TestArray arr;
  pub.publish(arr);

  Helper h;
  ros::Subscriber sub = n.subscribe("test", 1, &Helper::cb, &h);
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(h.count_, 0);
}

TEST(RoscppLatchingPublisher, latching)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<TestArray>("test", 1, true);
  TestArray arr;
  pub.publish(arr);

  Helper h;
  ros::Subscriber sub = n.subscribe("test", 1, &Helper::cb, &h);
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(h.count_, 1);

  ASSERT_STREQ((*h.last_msg_->__connection_header)["latching"].c_str(), "1");
}

TEST(RoscppLatchingPublisher, latchingMultipleSubscriptions)
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<TestArray>("test", 1, true);
  TestArray arr;
  pub.publish(arr);

  Helper h1, h2;
  ros::Subscriber sub1 = n.subscribe("test", 1, &Helper::cb, &h1);
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(h1.count_, 1);
  ASSERT_EQ(h2.count_, 0);

  ros::Subscriber sub2 = n.subscribe("test", 1, &Helper::cb, &h2);
  ros::spinOnce();

  ASSERT_EQ(h1.count_, 1);
  ASSERT_EQ(h2.count_, 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, g_node_name);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}

