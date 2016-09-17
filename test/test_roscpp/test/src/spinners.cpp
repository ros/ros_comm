/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Troy Straszheim */

/*
 * Spinny spinner basic tests.
 
 these do NOT ACTUALLY TEST ANYTHING as it is diffcult to restart all
 of ros in a unit test, and there is no way to test (that I can see
 right now) that certain error messages are emitted, nor any way to
 specify that a test should fail or return an error.  So this is just
 a placeholder to be run manually, one test at a time (via
 --gtest_filter) when the next problem occurs.  Those that end with
 'fail' actually success, but they send a fatal error message that
 you're trying to spin from multiple threads.
*/

#include <gtest/gtest.h>
#include "ros/spinner.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <boost/thread.hpp>

using namespace ros;

int argc_;
char** argv_;

void fire_shutdown(const ros::WallTimerEvent&) {
  ROS_INFO("Asking for shutdown");
  ros::shutdown();
}

#define DOIT()                                                  \
  ros::init(argc_, argv_, "test_spinners");                     \
  NodeHandle nh;                                                \
  ros::WallTimer t = nh.createWallTimer(ros::WallDuration(2.0), \
                                        &fire_shutdown);        \
  
TEST(Spinners, spin)
{
  DOIT();
  ros::spin(); // will block until ROS shutdown
}

TEST(Spinners, spinfail)
{
  DOIT();
  boost::thread th(boost::bind(&ros::spin));
  ros::WallDuration(0.1).sleep(); // wait for thread to be started

  EXPECT_THROW(ros::spin(), std::runtime_error);

  SingleThreadedSpinner ss;
  EXPECT_THROW(ros::spin(ss), std::runtime_error);
  EXPECT_THROW(ss.spin(), std::runtime_error);

  MultiThreadedSpinner ms;
  EXPECT_THROW(ros::spin(ms), std::runtime_error);
  EXPECT_THROW(ms.spin(), std::runtime_error);

  AsyncSpinner as(2);
  EXPECT_THROW(as.start(), std::runtime_error);

  ros::waitForShutdown();
}

TEST(Spinners, singlefail)
{
  DOIT();
  SingleThreadedSpinner ss;
  boost::thread th(boost::bind(&ros::spin, ss));
  ros::WallDuration(0.1).sleep(); // wait for thread to be started

  EXPECT_THROW(ros::spin(), std::runtime_error);

  SingleThreadedSpinner ss2;
  EXPECT_THROW(ros::spin(ss2), std::runtime_error);
  EXPECT_THROW(ss2.spin(), std::runtime_error);

  MultiThreadedSpinner ms;
  EXPECT_THROW(ros::spin(ms), std::runtime_error);
  EXPECT_THROW(ms.spin(), std::runtime_error);

  AsyncSpinner as(2);
  EXPECT_THROW(as.start(), std::runtime_error);

  ros::waitForShutdown();
}

TEST(Spinners, multi)
{
  DOIT();
  MultiThreadedSpinner ms;
  ros::spin(ms); // will block until ROS shutdown
}

TEST(Spinners, multifail)
{
  DOIT();
  MultiThreadedSpinner ms;
  boost::thread th(boost::bind(&ros::spin, ms));
  ros::WallDuration(0.1).sleep(); // wait for thread to be started

  SingleThreadedSpinner ss2;
  EXPECT_THROW(ros::spin(ss2), std::runtime_error);
  EXPECT_THROW(ss2.spin(), std::runtime_error);

  // running another multi-threaded spinner is allowed
  MultiThreadedSpinner ms2;
  ros::spin(ms2); // will block until ROS shutdown
}

TEST(Spinners, async)
{
  DOIT();
  AsyncSpinner as1(2);
  as1.start();

  // running another AsyncSpinner is allowed
  AsyncSpinner as2(2);
  as2.start();
  as2.stop();

  SingleThreadedSpinner ss;
  EXPECT_THROW(ros::spin(ss), std::runtime_error);
  EXPECT_THROW(ss.spin(), std::runtime_error);

  // running a multi-threaded spinner is allowed
  MultiThreadedSpinner ms;
  ros::spin(ms); // will block until ROS shutdown

  ros::waitForShutdown();
}


int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  argc_ = argc;
  argv_ = argv;
  return RUN_ALL_TESTS();
}
