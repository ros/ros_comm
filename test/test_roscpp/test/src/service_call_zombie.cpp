/*
 * Copyright (c) 2014 Max Schwarz
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

/*
 * Call a service which does not exist anymore.
 */

#include <string>

#include <gtest/gtest.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/service.h"
#include "ros/connection.h"
#include "ros/service_client.h"
#include <test_roscpp/TestStringString.h>

#include <stdio.h>

TEST(SrvCall, callPhantomService)
{
  ros::NodeHandle nh;
  for(int i = 0; i < 200; ++i)
  {
    ros::ServiceClient handle = nh.serviceClient<test_roscpp::TestStringString>("phantom_service");

    test_roscpp::TestStringString::Request req;
    test_roscpp::TestStringString::Request res;
    ASSERT_FALSE(handle.call(req, res));
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "service_call");
  ros::NodeHandle nh;

  sleep(10);

  int ret = RUN_ALL_TESTS();

  return ret;
}
