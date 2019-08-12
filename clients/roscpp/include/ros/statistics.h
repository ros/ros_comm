/*
 * Copyright (C) 2013-2014, Dariush Forouher
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_STATISTICS_H
#define ROSCPP_STATISTICS_H

#include "forwards.h"
#include "poll_set.h"
#include "common.h"
#include "publisher.h"
#include <ros/time.h>
#include "ros/subscription_callback_helper.h"
#include <map>

namespace ros
{

/**
 * \brief This class logs statistics data about a ROS connection and
 * publishs them periodically on a common topic.
 *
 * It provides a callback() function that has to be called everytime
 * a new message arrives on a topic.
 */
class ROSCPP_DECL StatisticsLogger
{
public:

  /**
   * Constructior
   */
  StatisticsLogger();

  /**
   * Actual initialization. Must be called before the first call to callback()
   */
  void init(const SubscriptionCallbackHelperPtr& helper);

  /**
   * Callback function. Must be called for every message received.
   */
  void callback(const boost::shared_ptr<M_string>& connection_header, const std::string& topic, const std::string& callerid, const SerializedMessage& m, const uint64_t& bytes_sent, const ros::Time& received_time, bool dropped);

private:

  // Range of window length, in seconds
  int max_window;
  int min_window;

  // Range of acceptable messages in window.
  // Window size will be adjusted if number of observed is
  // outside this range.
  int max_elements;
  int min_elements;

  bool enable_statistics;

  // remember, if this message type has a header
  bool hasHeader_;

  // frequency to publish statistics
  double pub_frequency_;

  // publisher for statistics data
  ros::Publisher pub_;

  struct StatData {
    // last time, we published /statistics data
    ros::Time last_publish;
    // arrival times of all messages within the current window
    std::list<ros::Time> arrival_time_list;
    // age of all messages within the current window (if available)
    std::list<ros::Duration> age_list;
    // number of dropped messages
    uint64_t dropped_msgs;
    // latest sequence number observered (if available)
    uint64_t last_seq;
    // latest total traffic volume observed
    uint64_t stat_bytes_last;
  };

  // storage for statistics data
  std::map<std::string, struct StatData> map_;
};

}

#endif
