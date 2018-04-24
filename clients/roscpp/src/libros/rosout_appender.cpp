/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "ros/rosout_appender.h"
#include "ros/this_node.h"
#include "ros/node_handle.h"
#include "ros/topic_manager.h"
#include "ros/advertise_options.h"
#include "ros/names.h"
#include "ros/param.h"

#include <tracetools/tracetools.h>
#include <rosgraph_msgs/Log.h>

namespace ros
{

ROSOutAppender::ROSOutAppender()
: shutting_down_(false)
, publish_thread_(boost::bind(&ROSOutAppender::logThread, this))
{
  AdvertiseOptions ops;
  ops.init<rosgraph_msgs::Log>(names::resolve("/rosout"), 0);
  ops.latch = true;
  SubscriberCallbacksPtr cbs(boost::make_shared<SubscriberCallbacks>());
  TopicManager::instance()->advertise(ops, cbs);
}

ROSOutAppender::~ROSOutAppender()
{
  shutting_down_ = true;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    queue_condition_.notify_all();
  }

  publish_thread_.join();
}

const std::string&  ROSOutAppender::getLastError() const
{
  return last_error_;
}

void ROSOutAppender::log(::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  rosgraph_msgs::LogPtr msg(boost::make_shared<rosgraph_msgs::Log>());

  msg->header.stamp = ros::Time::now();
  if (level == ros::console::levels::Debug)
  {
    msg->level = rosgraph_msgs::Log::DEBUG;
  }
  else if (level == ros::console::levels::Info)
  {
    msg->level = rosgraph_msgs::Log::INFO;
  }
  else if (level == ros::console::levels::Warn)
  {
    msg->level = rosgraph_msgs::Log::WARN;
  }
  else if (level == ros::console::levels::Error)
  {
    msg->level = rosgraph_msgs::Log::ERROR;
  }
  else if (level == ros::console::levels::Fatal)
  {
    msg->level = rosgraph_msgs::Log::FATAL;
  }
  msg->name = this_node::getName();
  msg->msg = str;
  msg->file = file;
  msg->function = function;
  msg->line = line;
  
  // check parameter server/cache for omit_topics flag
  // the same parameter is checked in rosout.py for the same purpose
  ros::param::getCached("/rosout_disable_topics_generation", disable_topics_);

  if (!disable_topics_){
    this_node::getAdvertisedTopics(msg->topics);
  }

  if (level == ::ros::console::levels::Fatal || level == ::ros::console::levels::Error)
  {
    last_error_ = str;
  }

  boost::mutex::scoped_lock lock(queue_mutex_);
  log_queue_.push_back(msg);
  queue_condition_.notify_all();
}

void ROSOutAppender::logThread()
{
  ros::trace::task_init("rosout appender");

  while (!shutting_down_)
  {
    V_Log local_queue;

    {
      boost::mutex::scoped_lock lock(queue_mutex_);

      if (shutting_down_)
      {
        return;
      }

      queue_condition_.wait(lock);

      if (shutting_down_)
      {
        return;
      }

      local_queue.swap(log_queue_);
    }

    V_Log::iterator it = local_queue.begin();
    V_Log::iterator end = local_queue.end();
    for (; it != end; ++it)
    {
      TopicManager::instance()->publish(names::resolve("/rosout"), *(*it));
    }
  }
}

} // namespace ros
