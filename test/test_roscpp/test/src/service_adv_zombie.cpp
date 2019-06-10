/*
 * Copyright (c) 2014 Max Schwarz <max.schwarz@uni-bonn.de>
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

// Advertise a service, then crash horribly and leaving a "zombie service"
// behind. Needed for service_call_zombie test.

#include <ros/ros.h>
#include <test_roscpp/TestStringString.h>

#include <stdlib.h>
#ifdef _WIN32
# include <windows.h>
#endif

bool srvCallback(test_roscpp::TestStringString::Request &,
                 test_roscpp::TestStringString::Response &res)
{
  res.str = "B";
  return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dying_node");

	ros::NodeHandle nh;
	ros::ServiceServer srv = nh.advertiseService("phantom_service", srvCallback);

	// Allow for some time for registering on the master
	for(int i = 0; i < 10; ++i)
	{
		ros::spinOnce();

#ifndef _WIN32
		usleep(100*1000);
#else
		Sleep(100);
#endif
	}

	// Exit immediately without calling any atexit hooks
	_Exit(0);
}
