/*********************************************************************
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
*   * Neither the name of Open Source Robotics Foundation, Inc. nor the
*     names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
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
********************************************************************/
#include <time.h>
#include <queue>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/scope_exit.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <ros/ros.h>
#include <ros/assert.h>
#include <topic_tools/shape_shifter.h>
#include <rosbag_msgs/SnapshotStatus.h>
#include "rosbag/snapshotter.h"

using std::string;
using boost::shared_ptr;
using ros::Time;

namespace rosbag
{
const ros::Duration SnapshotterTopicOptions::NO_DURATION_LIMIT = ros::Duration(-1);
const int32_t SnapshotterTopicOptions::NO_MEMORY_LIMIT = -1;
const ros::Duration SnapshotterTopicOptions::INHERIT_DURATION_LIMIT = ros::Duration(0);
const int32_t SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT = 0;

SnapshotterTopicOptions::SnapshotterTopicOptions(ros::Duration duration_limit, int32_t memory_limit)
  : duration_limit_(duration_limit), memory_limit_(memory_limit)
{
}

SnapshotterOptions::SnapshotterOptions(ros::Duration default_duration_limit, int32_t default_memory_limit,
                                     ros::Duration status_period)
  : default_duration_limit_(default_duration_limit)
  , default_memory_limit_(default_memory_limit)
  , status_period_(status_period)
  , topics_()
{
}

void SnapshotterOptions::addTopic(std::string const& topic, ros::Duration duration, int32_t memory)
{
  SnapshotterTopicOptions ops(duration, memory);
  topics_.insert(topics_t::value_type(topic, ops));
}

SnapshotterClientOptions::SnapshotterClientOptions() : action_(SnapshotterClientOptions::TRIGGER_WRITE)
{
}

SnapshotMessage::SnapshotMessage(topic_tools::ShapeShifter::ConstPtr _msg,
                                 boost::shared_ptr<ros::M_string> _connection_header, Time _time)
  : msg(_msg), connection_header(_connection_header), time(_time)
{
}

MessageQueue::MessageQueue(SnapshotterTopicOptions const& options) : options_(options), size_(0)
{
}

void MessageQueue::setSubscriber(shared_ptr<ros::Subscriber> sub)
{
  sub_ = sub;
}

void MessageQueue::fillStatus(rosgraph_msgs::TopicStatistics& status)
{
  boost::mutex::scoped_lock l(lock);
  if (!queue_.size())
    return;
  status.traffic = size_;
  status.delivered_msgs = queue_.size();
  status.window_start = queue_.front().time;
  status.window_stop = queue_.back().time;
}

void MessageQueue::clear()
{
  boost::mutex::scoped_lock l(lock);
  _clear();
}

void MessageQueue::_clear()
{
  queue_.clear();
  size_ = 0;
}

ros::Duration MessageQueue::duration() const
{
  // No duration if 0 or 1 messages
  if (queue_.size() <= 1)
    return ros::Duration();
  return queue_.back().time - queue_.front().time;
}

bool MessageQueue::preparePush(int32_t size, ros::Time const& time)
{
  // If new message is older than back of queue, time has gone backwards and buffer must be cleared
  if (!queue_.empty() and time < queue_.back().time)
  {
    ROS_WARN("Time has gone backwards. Clearing buffer for this topic.");
    _clear();
  }

  // The only case where message cannot be addded is if size is greater than limit
  if (options_.memory_limit_ > SnapshotterTopicOptions::NO_MEMORY_LIMIT && size > options_.memory_limit_)
    return false;

  // If memory limit is enforced, remove elements from front of queue until limit would be met once message is added
  if (options_.memory_limit_ > SnapshotterTopicOptions::NO_MEMORY_LIMIT)
    while (queue_.size() != 0 && size_ + size > options_.memory_limit_)
      _pop();

  // If duration limit is encforced, remove elements from front of queue until duration limit would be met once message
  // is added
  if (options_.duration_limit_ > SnapshotterTopicOptions::NO_DURATION_LIMIT && queue_.size() != 0)
  {
    ros::Duration dt = time - queue_.front().time;
    while (dt > options_.duration_limit_)
    {
      _pop();
      if (queue_.empty())
        break;
      dt = time - queue_.front().time;
    }
  }
  return true;
}
void MessageQueue::push(SnapshotMessage const& _out)
{
  boost::mutex::scoped_try_lock l(lock);
  if (!l.owns_lock())
  {
    ROS_ERROR("Failed to lock. Time %f", _out.time.toSec());
    return;
  }
  _push(_out);
}

SnapshotMessage MessageQueue::pop()
{
  boost::mutex::scoped_lock l(lock);
  return _pop();
}
void MessageQueue::_push(SnapshotMessage const& _out)
{
  int32_t size = _out.msg->size();
  // If message cannot be added without violating limits, it must be dropped
  if (not preparePush(size, _out.time))
    return;
  queue_.push_back(_out);
  // Add size of new message to running count to maintain correctness
  size_ += _out.msg->size();
}

SnapshotMessage MessageQueue::_pop()
{
  SnapshotMessage tmp = queue_.front();
  queue_.pop_front();
  //  Remove size of popped message to maintain correctness of size_
  size_ -= tmp.msg->size();
  return tmp;
}

MessageQueue::range_t MessageQueue::rangeFromTimes(Time const& start, Time const& stop)
{
  range_t::first_type begin = queue_.begin();
  range_t::second_type end = queue_.end();

  // Increment / Decrement iterators until time contraints are met
  if (not start.isZero())
  {
    while (begin != end and (*begin).time < start)
      ++begin;
  }
  if (not stop.isZero())
  {
    while (end != begin and (*(end - 1)).time > stop)
      --end;
  }
  return range_t(begin, end);
}

const int Snapshotter::QUEUE_SIZE = 10;

Snapshotter::Snapshotter(SnapshotterOptions const& options) : options_(options), recording_(true), writing_(false)
{
  status_pub_ = nh_.advertise<rosbag_msgs::SnapshotStatus>("snapshot_status", 10);
}

void Snapshotter::fixTopicOptions(SnapshotterTopicOptions& options)
{
  if (options.duration_limit_ == SnapshotterTopicOptions::INHERIT_DURATION_LIMIT)
    options.duration_limit_ = options_.default_duration_limit_;
  if (options.memory_limit_ == SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT)
    options.memory_limit_ = options_.default_memory_limit_;
}

bool Snapshotter::postfixFilename(string& file)
{
  size_t ind = file.rfind(".bag");
  // If requested ends in .bag, this is literal name do not append date
  if (ind != string::npos && ind == file.size() - 4)
  {
    return true;
  }
  // Otherwise treat as prefix and append datetime and extension
  file += timeAsStr() + ".bag";
  return true;
}

string Snapshotter::timeAsStr()
{
  std::stringstream msg;
  const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  boost::posix_time::time_facet* const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
  msg.imbue(std::locale(msg.getloc(), f));
  msg << now;
  return msg.str();
}

void Snapshotter::topicCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                         boost::shared_ptr<MessageQueue> queue)
{
  // If recording is paused (or writing), exit
  {
    boost::shared_lock<boost::upgrade_mutex> lock(state_lock_);
    if (!recording_)
    {
      return;
    }
  }

  // Pack message and metadata into SnapshotMessage holder
  SnapshotMessage out(msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), msg_event.getReceiptTime());
  queue->push(out);
}

void Snapshotter::subscribe(string const& topic, boost::shared_ptr<MessageQueue> queue)
{
  ROS_INFO("Subscribing to %s", topic.c_str());

  shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
  ros::SubscribeOptions ops;
  ops.topic = topic;
  ops.queue_size = QUEUE_SIZE;
  ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
  ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
  ops.helper =
      boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<topic_tools::ShapeShifter const>&> >(
          boost::bind(&Snapshotter::topicCB, this, _1, queue));
  *sub = nh_.subscribe(ops);
  queue->setSubscriber(sub);
}

bool Snapshotter::writeTopic(rosbag::Bag& bag, MessageQueue& message_queue, string const& topic,
                            rosbag_msgs::TriggerSnapshot::Request& req, rosbag_msgs::TriggerSnapshot::Response& res)
{
  // acquire lock for this queue
  boost::mutex::scoped_lock l(message_queue.lock);

  MessageQueue::range_t range = message_queue.rangeFromTimes(req.start_time, req.stop_time);

  // open bag if this the first valid topic and there is data
  if (not bag.isOpen() and range.second > range.first)
  {
    try
    {
      bag.open(req.filename, bagmode::Write);
    }
    catch (rosbag::BagException const& err)
    {
      res.success = false;
      res.message = string("failed to open bag: ") + err.what();
      return false;
    }
    ROS_INFO("Writing snapshot to %s", req.filename.c_str());
  }

  // write queue
  try
  {
    for (MessageQueue::range_t::first_type msg_it = range.first; msg_it != range.second; ++msg_it)
    {
      SnapshotMessage const& msg = *msg_it;
      bag.write(topic, msg.time, msg.msg, msg.connection_header);
    }
  }
  catch (rosbag::BagException const& err)
  {
    res.success = false;
    res.message = string("failed to write bag: ") + err.what();
  }
  return true;
}

bool Snapshotter::triggerSnapshotCb(rosbag_msgs::TriggerSnapshot::Request& req,
                                   rosbag_msgs::TriggerSnapshot::Response& res)
{
  if (not postfixFilename(req.filename))
  {
    res.success = false;
    res.message = "invalid";
    return true;
  }
  bool recording_prior;  // Store if we were recording prior to write to restore this state after write
  {
    boost::upgrade_lock<boost::upgrade_mutex> read_lock(state_lock_);
    recording_prior = recording_;
    if (writing_)
    {
      res.success = false;
      res.message = "Already writing";
      return true;
    }
    boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
    if (recording_prior)
      pause();
    writing_ = true;
  }

  // Ensure that state is updated when function exits, regardlesss of branch path / exception events
  BOOST_SCOPE_EXIT(&state_lock_, &writing_, recording_prior, this_)
  {
    // Clear buffers beacuase time gaps (skipped messages) may have occured while paused
    boost::unique_lock<boost::upgrade_mutex> write_lock(state_lock_);
    // Turn off writing flag and return recording to its state before writing
    writing_ = false;
    if (recording_prior)
      this_->resume();
  }
  BOOST_SCOPE_EXIT_END

  // Create bag
  Bag bag;

  // Write each selected topic's queue to bag file
  if (req.topics.size())
  {
    BOOST_FOREACH (std::string& topic, req.topics)
    {
      // Resolve and clean topic
      try
      {
        topic = ros::names::resolve(nh_.getNamespace(), topic);
      }
      catch (ros::InvalidNameException const& err)
      {
        ROS_WARN("Requested topic %s is invalid, skipping.", topic.c_str());
        continue;
      }

      // Find the message queue for this topic if it exsists
      buffers_t::iterator found = buffers_.find(topic);
      // If topic not found, error and exit
      if (found == buffers_.end())
      {
        ROS_WARN("Requested topic %s is not subscribed, skipping.", topic.c_str());
        continue;
      }
      MessageQueue& message_queue = *(*found).second;
      if (not writeTopic(bag, message_queue, topic, req, res))
        return true;
    }
  }
  // If topic list empty, record all buffered topics
  else
  {
    BOOST_FOREACH (buffers_t::value_type& pair, buffers_)
    {
      MessageQueue& message_queue = *(pair.second);
      std::string const& topic = pair.first;
      if (not writeTopic(bag, message_queue, topic, req, res))
        return true;
    }
  }

  // If no topics were subscribed/valid/contained data, this is considered a non-success
  if (not bag.isOpen())
  {
    res.success = false;
    res.message = res.NO_DATA;
    return true;
  }

  res.success = true;
  return true;
}

void Snapshotter::clear()
{
  BOOST_FOREACH (buffers_t::value_type& pair, buffers_)
  {
    pair.second->clear();
  }
}

void Snapshotter::pause()
{
  ROS_INFO("Buffering paused");
  recording_ = false;
}

void Snapshotter::resume()
{
  clear();
  recording_ = true;
  ROS_INFO("Buffering resumed and old data cleared.");
}

bool Snapshotter::enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  boost::upgrade_lock<boost::upgrade_mutex> read_lock(state_lock_);
  if (req.data && writing_)  // Cannot enable while writing
  {
    res.success = false;
    res.message = "cannot enable recording while writing.";
    return true;
  }
  // Obtain write lock and update state if requested state is different from current
  if (req.data and not recording_)
  {
    boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
    resume();
  }
  else if (not req.data and recording_)
  {
    boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
    pause();
  }
  res.success = true;
  return true;
}

void Snapshotter::publishStatus(ros::TimerEvent const& e)
{
  (void)e;  // Make your "unused variable" warnings a thing of the past with cast to void!
  // Don't bother doing computing if no one is subscribed
  if (!status_pub_.getNumSubscribers())
    return;

  // TODO: consider options to make this faster (caching and updating last status, having queues track their own status)
  rosbag_msgs::SnapshotStatus msg;
  {
    boost::shared_lock<boost::upgrade_mutex> lock(state_lock_);
    msg.enabled = recording_;
  }
  std::string node_id = ros::this_node::getName();
  BOOST_FOREACH (buffers_t::value_type& pair, buffers_)
  {
    rosgraph_msgs::TopicStatistics status;
    status.node_sub = node_id;
    status.topic = pair.first;
    pair.second->fillStatus(status);
    msg.topics.push_back(status);
  }

  status_pub_.publish(msg);
}

int Snapshotter::run()
{
  if (!nh_.ok())
    return 0;

  // Create the queue for each topic and set up the subscriber to add to it on new messages
  BOOST_FOREACH (SnapshotterOptions::topics_t::value_type& pair, options_.topics_)
  {
    string topic = ros::names::resolve(nh_.getNamespace(), pair.first);
    fixTopicOptions(pair.second);
    shared_ptr<MessageQueue> queue;
    queue.reset(new MessageQueue(pair.second));
    std::pair<buffers_t::iterator, bool> res = buffers_.insert(buffers_t::value_type(topic, queue));
    ROS_ASSERT_MSG(res.second, "failed to add %s to topics. Perhaps it is a duplicate?", topic.c_str());
    subscribe(topic, queue);
  }

  // Now that subscriptions are setup, setup service servers for writing and pausing
  trigger_snapshot_server_ = nh_.advertiseService("trigger_snapshot", &Snapshotter::triggerSnapshotCb, this);
  enable_server_ = nh_.advertiseService("enable_snapshot", &Snapshotter::enableCB, this);

  // Start timer to publish status regularly
  if (options_.status_period_ > ros::Duration(0))
    status_timer_ = nh_.createTimer(options_.status_period_, &Snapshotter::publishStatus, this);

  // Use multiple callback threads
  ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
  spinner.spin();                        // spin() will not return until the node has been shutdown
  return 0;
}

SnapshotterClient::SnapshotterClient()
{
}

int SnapshotterClient::run(SnapshotterClientOptions const& opts)
{
  if (opts.action_ == SnapshotterClientOptions::TRIGGER_WRITE)
  {
    ros::ServiceClient client = nh_.serviceClient<rosbag_msgs::TriggerSnapshot>("trigger_snapshot");
    if (not client.exists())
    {
      ROS_ERROR("Service %s does not exist. Is snapshot running in this namespace?", "trigger_snapshot");
      return 1;
    }
    rosbag_msgs::TriggerSnapshotRequest req;
    req.topics = opts.topics_;
    // Prefix mode
    if (opts.filename_.empty())
    {
      req.filename = opts.prefix_;
      size_t ind = req.filename.rfind(".bag");
      if (ind != string::npos && ind == req.filename.size() - 4)
        req.filename.erase(ind);
    }
    else
    {
      req.filename = opts.filename_;
      size_t ind = req.filename.rfind(".bag");
      if (ind == string::npos || ind != req.filename.size() - 4)
        req.filename += ".bag";
    }

    // Resolve filename relative to clients working directory to avoid confusion
    if (req.filename.empty()) // Special case of no specified file, ensure still in working directory of client
      req.filename = "./";
    boost::filesystem::path p(boost::filesystem::system_complete(req.filename));
    req.filename = p.string();

    rosbag_msgs::TriggerSnapshotResponse res;
    if (not client.call(req, res))
    {
      ROS_ERROR("Failed to call service");
      return 1;
    }
    if (not res.success)
    {
      ROS_ERROR("%s", res.message.c_str());
      return 1;
    }
    return 0;
  }
  else if (opts.action_ == SnapshotterClientOptions::PAUSE || opts.action_ == SnapshotterClientOptions::RESUME)
  {
    ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("enable_snapshot");
    if (not client.exists())
    {
      ROS_ERROR("Service %s does not exist. Is snapshot running in this namespace?", "enable_snapshot");
      return 1;
    }
    std_srvs::SetBoolRequest req;
    req.data = (opts.action_ == SnapshotterClientOptions::RESUME);
    std_srvs::SetBoolResponse res;
    if (not client.call(req, res))
    {
      ROS_ERROR("Failed to call service.");
      return 1;
    }
    if (not res.success)
    {
      ROS_ERROR("%s", res.message.c_str());
      return 1;
    }
    return 0;
  }
  else
  {
    ROS_ASSERT_MSG(false, "Invalid value of enum.");
    return 1;
  }
}

}  // namespace rosbag
