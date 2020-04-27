// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Yuki Furuta, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/
/*
 * synchronized_throttle.cpp
 * Author: Yuki Furuta <me@furushchev.ru>
 */

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include "topic_tools/shape_shifter_stamped.h"


static const int MAX_SYNC_NUM = 8;
static const char* USAGE = "\nusage: \n"\
  "  synchronized_throttle MSGS_PER_SEC IN_TOPIC0 IN_TOPIC1 [IN_TOPIC2]...[IN_TOPIC7]\n\n"\
  "  This program will throttle messages like topic_tools/throttle\n"\
  "  and also synchronize timestamps for each throttled messages.\n"\
  "  Due to the limit of message_filters, maximum number of input topics are limited up to 8.\n\n";


namespace topic_tools
{
class SynchronizedThrottle
{
 public:
  typedef message_filters::sync_policies::ExactTime<
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped> SyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped> AsyncPolicy;

  SynchronizedThrottle(const std::vector<std::string> &input_topics,
                       const ros::Duration &rate)
    : nh_(""), pnh_("~"), input_topics_(input_topics), rate_(rate),
      subscribed_(false), advertised_(false) {

    pnh_.param("enable_warning", enable_warning_, true);
    pnh_.param("wall_clock", use_wall_time_, false);
    pnh_.param("lazy", lazy_, false);
    pnh_.param("approximate_sync", approximate_sync_, false);
    pnh_.param("queue_size", queue_size_, 100);
    pnh_.param("suffix", suffix_, std::string());
    if (suffix_.empty()) {
      suffix_ = "throttled";
    }

    const size_t n_topics = input_topics_.size();
    check_sub_.resize(n_topics);
    sub_.resize(n_topics);
    pub_.resize(n_topics);
    for (size_t i = 0; i < n_topics; ++i)
    {
      /* Subscribe input topics so to receive at least one message,
         in order to identify message definition for each input topics. */
      ROS_DEBUG_STREAM("Subscribing " << input_topics_[i]);
      check_sub_[i] = pnh_.subscribe<topic_tools::ShapeShifterStamped>(
        input_topics_[i], 1,
        boost::bind(&SynchronizedThrottle::checkCallback, this, _1, i));
      sub_[i].reset(new message_filters::Subscriber<topic_tools::ShapeShifterStamped>());
    }

    if (enable_warning_)
    {
      /* Since all output messages are synchronized,
         even there is one topic where messages don't come, it causes malfunction.
         This timer checks input topics that has no message received
         and displays warning messages to screen.
      */
      check_timer_ = pnh_.createWallTimer(
        ros::WallDuration(5), &SynchronizedThrottle::checkAdvertisedTimerCallback,
        this, false);
    }

    ROS_DEBUG_STREAM("Publishing at rate " << rate_.toSec() << " Hz");
  }

 protected:
  void checkAdvertisedTimerCallback(const ros::WallTimerEvent&) {
    for (size_t i = 0; i < pub_.size(); ++i)
    {
      if (!pub_[i])
      {
        ROS_WARN_STREAM(input_topics_[i] << " is not yet published");
      }
    }
    if (advertised_)
    {
      check_timer_.stop();
    }
  }

  void checkCallback(
    const topic_tools::ShapeShifterStamped::ConstPtr &msg,
    const size_t index) {

    boost::mutex::scoped_lock lock(mutex_);

    ROS_DEBUG_STREAM("check callback: " << index);
    ROS_DEBUG_STREAM(" name: " << input_topics_[index]);
    ROS_DEBUG_STREAM(" type: " << msg->getDataType());
    ROS_DEBUG_STREAM(" md5: " << msg->getMD5Sum());

    check_sub_[index].shutdown();

    // advertise
    ros::SubscriberStatusCallback connect_cb
      = boost::bind(&SynchronizedThrottle::connectCb, this);
    ros::SubscriberStatusCallback disconnect_cb
      = boost::bind(&SynchronizedThrottle::disconnectCb, this);
    std::string output_topic = input_topics_[index] + "/" + suffix_;
    ros::AdvertiseOptions options(
      output_topic, 1,
      msg->getMD5Sum(), msg->getDataType(), msg->getMessageDefinition(),
      connect_cb, disconnect_cb);
    options.latch = false;
    pub_[index] = pnh_.advertise(options);

    // check if all are advertised
    bool all_advertised = true;
    for (size_t i = 0; i < pub_.size(); ++i)
    {
      if (!pub_[i]) all_advertised = false;
    }
    if (all_advertised)
    {
      ROS_DEBUG("All Advertised");
      advertised_ = true;
      if (!subscribed_)
      {
        if (lazy_)
        {
          for (size_t i = 0; i < pub_.size(); ++i)
          {
            if (pub_[i].getNumSubscribers() > 0) {
              subscribe();
              subscribed_ = true;
              break;
            }
          }
        } else {
          subscribe();
          subscribed_ = true;
        }
      }
    }
  }

  void subscribe() {
    ROS_DEBUG("subscribe");

    const size_t n_topics = input_topics_.size();

    for (size_t i = 0; i < n_topics; ++i)
    {
      sub_[i]->subscribe(pnh_, input_topics_[i], 1);
    }
    if (n_topics < MAX_SYNC_NUM)
    {
      sub_[0]->registerCallback(
        boost::bind(&SynchronizedThrottle::fillNullMessage, this, _1));
    }

    if (approximate_sync_)
    {
      async_ = boost::make_shared<message_filters::Synchronizer<AsyncPolicy> >(queue_size_);

      switch (n_topics)
      {
      case 2:
        async_->connectInput(*sub_[0], *sub_[1], null_, null_,
                             null_, null_, null_, null_);
        break;
      case 3:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], null_,
                             null_, null_, null_, null_);
        break;
      case 4:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             null_, null_, null_, null_);
        break;
      case 5:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], null_, null_, null_);
        break;
      case 6:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], null_, null_);
        break;
      case 7:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], *sub_[6], null_);
        break;
      case 8:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], *sub_[6], *sub_[7]);
        break;
      default:
        ROS_FATAL("Unhandled error");
        return;
      }
      async_->registerCallback(
        boost::bind(&SynchronizedThrottle::inputCallback, this,
                    _1, _2, _3, _4, _5, _6, _7, _8));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);

      switch (n_topics)
      {
      case 2:
        sync_->connectInput(*sub_[0], *sub_[1], null_, null_,
                            null_, null_, null_, null_);
        break;
      case 3:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], null_,
                            null_, null_, null_, null_);
        break;
      case 4:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            null_, null_, null_, null_);
        break;
      case 5:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], null_, null_, null_);
        break;
      case 6:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], *sub_[5], null_, null_);
        break;
      case 7:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], *sub_[5], *sub_[6], null_);
        break;
      case 8:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], *sub_[5], *sub_[6], *sub_[7]);
        break;
      default:
        ROS_FATAL("Unhandled error");
        return;
      }
      sync_->registerCallback(
        boost::bind(&SynchronizedThrottle::inputCallback, this,
                    _1, _2, _3, _4, _5, _6, _7, _8));
    }
  }

  void unsubscribe() {
    ROS_DEBUG("unsubscribe");

    for (size_t i = 0; i < sub_.size(); ++i) {
      sub_[i]->unsubscribe();
    }
  }

  void connectCb() {
    ROS_DEBUG("connectCb");

    boost::mutex::scoped_lock lock(mutex_);

    if (advertised_ && !subscribed_)
    {
      for (size_t i = 0; i < pub_.size(); ++i)
      {
        if (pub_[i].getNumSubscribers() > 0) {
          subscribe();
          subscribed_ = true;
          break;
        }
      }
    }
  }

  void disconnectCb() {
    ROS_DEBUG("disconnectCb");

    boost::mutex::scoped_lock lock(mutex_);

    if (lazy_ && subscribed_)
    {
      bool need_unsubscribe = true;
      for (size_t i = 0; i < pub_.size(); ++i)
      {
        if (pub_[i].getNumSubscribers() > 0)
        {
          need_unsubscribe = false;
          break;
        }
      }
      if (need_unsubscribe)
      {
        unsubscribe();
        subscribed_ = false;
      }
    }
  }

  void fillNullMessage(
    const topic_tools::ShapeShifterStamped::ConstPtr& msg) {
    ROS_DEBUG("fillNullMessage");
    null_.add(msg);
  }

  void inputCallback(
      const topic_tools::ShapeShifterStamped::ConstPtr& msg0,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg1,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg2,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg3,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg4,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg5,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg6,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg7) {

    ROS_DEBUG("input callback");
    boost::mutex::scoped_lock lock(mutex_);

    ros::Time now;
    if (use_wall_time_) now.fromSec(ros::WallTime::now().toSec());
    else                now = ros::Time::now();

    // detect time jump back
    if (last_stamp_ > now)
    {
      ROS_WARN("Detected jump back in time. last_stamp_ is overwritten.");
      last_stamp_ = now;
    }

    // throttle
    if (rate_ <= ros::Duration(0.0) ||
        (now - last_stamp_) < rate_)
      return;

    // publish
    topic_tools::ShapeShifterStamped::ConstPtr msgs[] =
      {msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7};

    for (size_t i = 0; i < pub_.size(); ++i)
    {
      if (pub_[i].getNumSubscribers() > 0)
      {
        pub_[i].publish(msgs[i]);
      }
    }
    last_stamp_ = now;

  }

  boost::mutex mutex_;
  ros::NodeHandle nh_, pnh_;
  ros::WallTimer check_timer_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  boost::shared_ptr<message_filters::Synchronizer<AsyncPolicy> > async_;
  std::vector<ros::Subscriber> check_sub_;
  std::vector<boost::shared_ptr<message_filters::Subscriber<topic_tools::ShapeShifterStamped> > > sub_;
  message_filters::PassThrough<topic_tools::ShapeShifterStamped> null_;
  std::vector<ros::Publisher> pub_;

  std::vector<std::string> input_topics_;
  ros::Duration rate_;
  std::string suffix_;
  ros::Time last_stamp_;
  bool subscribed_;
  bool advertised_;
  bool enable_warning_;
  bool use_wall_time_;
  bool approximate_sync_;
  bool lazy_;
  int queue_size_;
};
}



int main(int argc, char** argv)
{
  if (argc < 4)
  {
    puts(USAGE);
    return 1;
  }

  ros::init(argc, argv, "synchronized_throttle",
            ros::init_options::AnonymousName);

  // parse arguments
  ros::Duration rate = ros::Duration(1.0 / atof(argv[1]));
  std::vector<std::string> input_topics;
  for (int i = 2; i < argc; ++i)
  {
    input_topics.push_back(std::string(argv[i]));
  }

  if (input_topics.size() > MAX_SYNC_NUM)
  {
    ROS_ERROR_STREAM("Found more than " << MAX_SYNC_NUM << " input topics which is not supported");
    return 1;
  }

  topic_tools::SynchronizedThrottle t(input_topics, rate);
  ros::spin();

  return 0;
}
