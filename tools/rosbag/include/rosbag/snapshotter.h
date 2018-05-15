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
#ifndef ROSBAG_SNAPSHOTER_H
#define ROSBAG_SNAPSHOTER_H

#include <deque>
#include <map>
#include <string>
#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag_msgs/TriggerSnapshot.h>
#include <std_srvs/SetBool.h>
#include <topic_tools/shape_shifter.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <rosbag_msgs/SnapshotStatus.h>
#include "rosbag/bag.h"
#include "rosbag/macros.h"

namespace rosbag
{
class ROSBAG_DECL Snapshotter;

/* Configuration for a single topic in the Snapshotter node. Holds
 * the buffer limits for a topic by duration (time difference between newest and oldest message)
 * and memory usage, in bytes.
 */
struct ROSBAG_DECL SnapshotterTopicOptions
{
  // When the value of duration_limit_, do not truncate the buffer no matter how large the duration is
  static const ros::Duration NO_DURATION_LIMIT;
  // When the value of memory_limit_, do not trunctate the buffer no matter how much memory it consumes (DANGROUS)
  static const int32_t NO_MEMORY_LIMIT;
  // When the value of duration_limit_, inherit the limit from the node's configured default
  static const ros::Duration INHERIT_DURATION_LIMIT;
  // When the value of memory_limit_, inherit the limit from the node's configured default
  static const int32_t INHERIT_MEMORY_LIMIT;

  // Maximum difference in time from newest and oldest message in buffer before older messages are removed
  ros::Duration duration_limit_;
  // Maximum memory usage of the buffer before older messages ar eremoved
  int32_t memory_limit_;

  SnapshotterTopicOptions(ros::Duration duration_limit = INHERIT_DURATION_LIMIT,
                         int32_t memory_limit = INHERIT_MEMORY_LIMIT);
};

/* Configuration for the Snapshotter node. Contains default limits for memory and duration
 * and a map of topics to their limits which may override the defaults.
 */
struct ROSBAG_DECL SnapshotterOptions
{
  // Duration limit to use for a topic's buffer if one is not specified
  ros::Duration default_duration_limit_;
  // Memory limit to use for a topic's buffer if one is not specified
  int32_t default_memory_limit_;
  // Period between publishing topic status messages. If <= ros::Duration(0), don't publish status
  ros::Duration status_period_;
  typedef std::map<std::string, SnapshotterTopicOptions> topics_t;
  // Provides list of topics to snapshot and their limit configurations
  topics_t topics_;

  SnapshotterOptions(ros::Duration default_duration_limit = ros::Duration(30), int32_t default_memory_limit = -1,
                    ros::Duration status_period = ros::Duration(1));

  // Add a new topic to the configuration
  void addTopic(std::string const& topic, ros::Duration duration_limit = SnapshotterTopicOptions::INHERIT_DURATION_LIMIT,
                int32_t memory_limit = SnapshotterTopicOptions::INHERIT_MEMORY_LIMIT);
};

/* Stores a buffered message of an ambiguous type and it's associated metadata (time of arrival, connection data),
 * for later writing to disk
 */
struct ROSBAG_DECL SnapshotMessage
{
  SnapshotMessage(topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header,
                  ros::Time _time);
  topic_tools::ShapeShifter::ConstPtr msg;
  boost::shared_ptr<ros::M_string> connection_header;
  // ROS time when messaged arrived (does not use header stamp)
  ros::Time time;
};

/* Stores a queue of buffered messages for a single topic ensuring
 * that the duration and memory limits are respected by truncating
 * as needed on push() operations.
 */
class ROSBAG_DECL MessageQueue
{
  friend Snapshotter;

private:
  // Locks access to size_ and queue_
  boost::mutex lock;
  // Stores limits on buffer size and duration
  SnapshotterTopicOptions options_;
  // Current total size of the queue, in bytes
  int64_t size_;
  typedef std::deque<SnapshotMessage> queue_t;
  queue_t queue_;
  // Subscriber to the callback which uses this queue
  boost::shared_ptr<ros::Subscriber> sub_;

public:
  MessageQueue(SnapshotterTopicOptions const& options);
  // Add a new message to the internal queue if possible, truncating the front of the queue as needed to enforce limits
  void push(SnapshotMessage const& msg);
  // Removes the message at the front of the queue (oldest) and returns it
  SnapshotMessage pop();
  // Returns the time difference between back and front of queue, or 0 if size <= 1
  ros::Duration duration() const;
  // Clear internal buffer
  void clear();
  // Store the subscriber for this topic's queue internaly so it is not deleted
  void setSubscriber(boost::shared_ptr<ros::Subscriber> sub);
  // Put data about oldest/newest message time, message count, and buffersize into status message
  void fillStatus(rosgraph_msgs::TopicStatistics& status);
  typedef std::pair<queue_t::const_iterator, queue_t::const_iterator> range_t;
  // Get a begin and end iterator into the buffer respecting the start and end timestamp constraints
  range_t rangeFromTimes(ros::Time const& start, ros::Time const& end);

private:
  // Internal push whitch does not obtain lock
  void _push(SnapshotMessage const& msg);
  // Internal pop which does not obtain lock
  SnapshotMessage _pop();
  // Internal clear which does not obtain lock
  void _clear();
  // Truncate front of queue as needed to fit a new message of specified size and time. Returns False if this is
  // impossible.
  bool preparePush(int32_t size, ros::Time const& time);
};

/* Snapshotter node. Maintains a circular buffer of the most recent messages from configured topics
 * while enforcing limits on memory and duration. The node can be triggered to write some or all
 * of these buffers to a bag file via a service call. Useful in live testing scenerios where interesting
 * data may be produced before a user has the oppurtunity to "rosbag record" the data.
 */
class ROSBAG_DECL Snapshotter
{
public:
  Snapshotter(SnapshotterOptions const& options);
  // Sets up callbacks and spins until node is killed
  int run();

private:
  // Subscribe queue size for each topic
  static const int QUEUE_SIZE;
  SnapshotterOptions options_;
  typedef std::map<std::string, boost::shared_ptr<MessageQueue> > buffers_t;
  buffers_t buffers_;
  // Locks recording_ and writing_ states.
  boost::upgrade_mutex state_lock_;
  // True if new messages are being written to the internal buffer
  bool recording_;
  // True if currently writing buffers to a bag file
  bool writing_;
  ros::NodeHandle nh_;
  ros::ServiceServer trigger_snapshot_server_;
  ros::ServiceServer enable_server_;
  ros::Publisher status_pub_;
  ros::Timer status_timer_;

  // Replace individual topic limits with node defaults if they are flagged for it (see SnapshotterTopicOptions)
  void fixTopicOptions(SnapshotterTopicOptions& options);
  // If file is "prefix" mode (doesn't end in .bag), append current datetime and .bag to end
  bool postfixFilename(std::string& file);
  /// Return current local datetime as a string such as 2018-05-22-14-28-51. Used to generate bag filenames
  std::string timeAsStr();
  // Clear the internal buffers of all topics. Used when resuming after a pause to avoid time gaps
  void clear();
  // Subscribe to one of the topics, setting up the callback to add to the respective queue
  void subscribe(std::string const& topic, boost::shared_ptr<MessageQueue> queue);
  // Called on new message from any configured topic. Adds to queue for that topic
  void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
               boost::shared_ptr<MessageQueue> queue);
  // Service callback, write all of part of the internal buffers to a bag file according to request parameters
  bool triggerSnapshotCb(rosbag_msgs::TriggerSnapshot::Request& req, rosbag_msgs::TriggerSnapshot::Response& res);
  // Service callback, enable or disable recording (storing new messages into queue). Used to pause before writing
  bool enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  // Set recording_ to false and do nessesary cleaning, CALLER MUST OBTAIN LOCK
  void pause();
  // Set recording_ to true and do nesessary cleaning, CALLER MUST OBTAIN LOCK
  void resume();
  // Publish status containing statistics of currently buffered topics and other state
  void publishStatus(ros::TimerEvent const& e);
  // Write the parts of message_queue within the time constraints of req to the queue
  // If returns false, there was an error opening/writing the bag and an error message was written to res.message
  bool writeTopic(rosbag::Bag& bag, MessageQueue& message_queue, std::string const& topic,
                  rosbag_msgs::TriggerSnapshot::Request& req, rosbag_msgs::TriggerSnapshot::Response& res);
};

// Configuration for SnapshotterClient
struct ROSBAG_DECL SnapshotterClientOptions
{
  SnapshotterClientOptions();
  enum Action
  {
    TRIGGER_WRITE,
    PAUSE,
    RESUME
  };
  // What to do when SnapshotterClient.run is called
  Action action_;
  // List of topics to write when action_ == TRIGGER_WRITE. If empty, write all buffered topics.
  std::vector<std::string> topics_;
  // Name of file to write to when action_ == TRIGGER_WRITE, relative to snapshot node. If empty, use prefix
  std::string filename_;
  // Prefix of the name of file written to when action_ == TRIGGER_WRITE.
  std::string prefix_;
};

// Node used to call services which interface with the snapshotter node to trigger write, pause, and resume
class ROSBAG_DECL SnapshotterClient
{
public:
  SnapshotterClient();
  int run(SnapshotterClientOptions const& opts);

private:
  ros::NodeHandle nh_;
};

}  // namespace rosbag

#endif
