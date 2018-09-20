/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Open Source Robotics Foundation, Inc.
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/test_perf_benchmark/common.h"

#include <string>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <vector>
#include <algorithm>

static ShmData* addr_;
static int shm_fd_ = -1;
static int count = 0;
char *message = NULL;

void UpdataShmData(const struct timespec& ts_end)
{
  flock(shm_fd_, LOCK_EX);
  addr_->ts_transmission_end.tv_sec = ts_end.tv_sec;
  addr_->ts_transmission_end.tv_nsec = ts_end.tv_nsec;
  addr_->need_subscriber_count--;
  flock(shm_fd_, LOCK_UN);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  struct timespec _ts_transmission_end;
  timespec_get(&_ts_transmission_end, TIME_UTC);

  assert(strcmp(message, msg->data.c_str()) == 0);

  UpdataShmData(_ts_transmission_end);

  if (++count == addr_->send_times)
  {
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Usage:\n\t./listener [nodename]\n" << std::endl;
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, argv[1]);

  ros::NodeHandle n;

  /*
   * If a TCP transport is used, set tcpNoDelay to true to provide a potentially
   * lower-latency connection.
   */
  ros::Subscriber sub = n.subscribe("chatter_perf_test", 1000, chatterCallback,
                            ros::TransportHints().tcpNoDelay());

  shm_fd_ = open(SHM_MAP_FILE, O_RDWR);
  if (shm_fd_ < 0)
  {
    ROS_ERROR("open file %s error", SHM_MAP_FILE);
    return EXIT_FAILURE;
  }
  addr_ = (ShmData*) mmap(NULL, sizeof(ShmData), PROT_WRITE | PROT_READ,
                    MAP_SHARED | MAP_POPULATE, shm_fd_, 0);
  if (addr_ == NULL)
  {
    ROS_ERROR("mmap error");
    return EXIT_FAILURE;
  }

  long int buffer_size = addr_->msg_size;
  message = new char[buffer_size];
  memset(message, '0', buffer_size);
  message[buffer_size - 1] = '\0';

  flock(shm_fd_, LOCK_EX);
  addr_->need_subscriber_count--;
  flock(shm_fd_, LOCK_UN);

  ros::spin();

  if (addr_ != NULL)
  {
    munmap(addr_, sizeof (ShmData));
  }

  if (shm_fd_ != -1)
  {
    close(shm_fd_);
  }

  delete [] message;

  return 0;
}
