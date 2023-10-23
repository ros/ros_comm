/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: David Gossow */

/*
 * Call ros::start() explicitly, but never call ros::shutdown().
 * ros::shutdown should be called automatically in this case.
 */

#include <cstdlib>
#include <ros/ros.h>

namespace ros
{
namespace console
{
extern bool g_shutting_down;
}
}

namespace
{

enum TestId
{
  InitOnly = 0,
  InitAndStart = 1
};

TestId test_id = InitOnly;

void atexitCallback()
{
  bool hasError = false;

  switch (test_id)
  {
    case InitOnly:
      if (!ros::ok())
      {
        std::cerr << "ERROR: ros::ok() returned false, although ros::shutdown has not been called!" << std::endl;
        hasError = true;
      }
      if (ros::isShuttingDown())
      {
        std::cerr << "ERROR: ros::isShuttingDown() returned true, although ros::shutdown has not been called!" << std::endl;
        hasError = true;
      }
      if (ros::isStarted())
      {
        std::cerr << "ERROR: ros::isStarted() returned true, although ros::start has not been called!" << std::endl;
        hasError = true;
      }
      break;
    case InitAndStart:
      if (ros::ok())
      {
        std::cerr << "ERROR: ros::ok() returned true after ros::shutdown should have been automatically called!" << std::endl;
        hasError = true;
      }
      if (!ros::isShuttingDown())
      {
        std::cerr << "ERROR: ros::isShuttingDown() returned false after ros::shutdown should have been automatically called!" << std::endl;
        hasError = true;
      }
      if (ros::isStarted())
      {
        std::cerr << "ERROR: ros::isStarted() returned true after ros::shutdown should have been automatically called!" << std::endl;
        hasError = true;
      }
      break;
  }

  if (!ros::isInitialized())
  {
    std::cerr << "ERROR: ros::isInitialized() returned false, although ros::init was called!" << std::endl;
    std::cerr << "Due to legacy reasons, it should return true, even after ROS has been de-initialized." << std::endl;
    hasError = true;
  }

  if (!ros::console::g_shutting_down)
  {
    std::cerr << "ERROR: ros::console::g_shutting_down returned false, but it should have been automatically shut down." << std::endl;
    hasError = true;
  }

  if (hasError)
  {
    std::_Exit(1);
  }
}
}

int
main(int argc, char** argv)
{
  if ( argc > 1 )
  {
    test_id = static_cast<TestId>(atoi(argv[1]));
  }

  // Register atexit callbak which will be executed after ROS has been de-initialized.
  if (atexit(atexitCallback) != 0)
  {
    std::cerr << "Failed to register atexit callback." << std::endl;
    return 1;
  }

  switch (test_id)
  {
    case InitOnly:
      // Test case 0: Call ros::init() explicitly, but never call ros::shutdown().
      // ros::deInit should be called automatically in this case.
      ros::init(argc, argv, "missing_call_to_shutdown" );
      break;
    case InitAndStart:
      // Test case 1: Call ros::init() and ros::start() explicitly, but never call ros::shutdown().
      // ros::shutdown should be called automatically in this case.
      ros::init(argc, argv, "missing_call_to_shutdown" );
      ros::start();
      break;
    default:
      std::cerr << "Invalid test id: " << test_id << std::endl;
      return 1;
      break;
  }

  if (!ros::ok())
  {
    std::cerr << "Failed to start ROS." << std::endl;
    return 1;
  }
  
  return 0;
}
