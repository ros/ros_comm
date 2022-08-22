/*
 * Copyright (C) 2022, Torc Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Torc Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include <cstdio>
#include "ros/statistics_manager.h"
#include "ros/node_handle.h"
#include "ros/this_node.h"

using namespace std; // sigh

namespace ros
{

const StatisticsManagerPtr& StatisticsManager::instance()
{
  static StatisticsManagerPtr statistics_manager = boost::make_shared<StatisticsManager>();
  return statistics_manager;
}

StatisticsManager::StatisticsManager()
{

}

StatisticsManager::~StatisticsManager()
{
  shutdown();
}

void StatisticsManager::addToQueue(rosgraph_msgs::TopicStatistics msg)
{
  if(!shutting_down_)
  {
    boost::mutex::scoped_lock lock(statistics_queue_mutex_);
    statistics_queue_.push_back(msg);
  }
}

void StatisticsManager::threadFunc()
{
  while(!shutting_down_)
  {
    if (!statistics_queue_.empty() && !pub_.getTopic().length())
    {
      ros::NodeHandle n("~");
      // We do not want to advertise in the constructor because that will advertise even if enable_statistics is unset or set to false. 
      pub_ = n.advertise<rosgraph_msgs::TopicStatistics>("/statistics", 1);
    }
    boost::mutex::scoped_lock lock(statistics_queue_mutex_);
    while(!statistics_queue_.empty() && pub_.getTopic().length())
    {
      auto msg = statistics_queue_.front();
      pub_.publish(msg);
      statistics_queue_.pop_front();
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
  }
}

void StatisticsManager::start()
{
  shutting_down_ = false;
  thread_ = boost::thread(&StatisticsManager::threadFunc, this);
}

void StatisticsManager::shutdown()
{
  shutting_down_ = true;
  thread_.join();
}

} // namespace ros

