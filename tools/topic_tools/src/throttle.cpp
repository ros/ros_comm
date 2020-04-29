///////////////////////////////////////////////////////////////////////////////
// throttle will transform a topic to have a limited number of bytes per second
//
// Copyright (C) 2009, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////


// this could be made a lot smarter by trying to analyze and predict the
// message stream density, etc., rather than just being greedy and stuffing
// the output as fast as it can. 

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <ostream>
#include <set>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"
#include <XmlRpcValue.h>

using std::string;
using std::vector;
using std::deque;
using namespace topic_tools;

// TODO: move all these globals into a reasonable local scope
ros::NodeHandle *g_node = NULL;
uint32_t g_bps = 0; // bytes per second, not bits!
ros::Duration g_period; // minimum inter-message period
double g_window = 1.0; // 1 second window for starters
bool g_advertised = false;
string g_output_topic;
string g_input_topic;
ros::Publisher g_pub;
ros::Subscriber* g_sub;
bool g_use_messages;
ros::Time g_last_time;
bool g_use_wallclock;
bool g_lazy;
bool g_force_latch = false;
bool g_force_latch_value = true;
ros::TransportHints g_th;
ros::WallDuration g_wait_for_subscribers_timeout{ 1.0 };
ros::WallDuration g_wait_for_subscribers_connect_time{ 0.0 };
vector<string> g_wait_for_subscribers;

class Sent
{
public:
  double t;
  uint32_t len;
  Sent(double _t, uint32_t _len) : t(_t), len(_len) { }
};
deque<Sent> g_sent;

void conn_cb(const ros::SingleSubscriberPublisher&);
void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event);

void subscribe()
{
  g_sub = new ros::Subscriber(g_node->subscribe(g_input_topic, 10, &in_cb, g_th));
}

void conn_cb(const ros::SingleSubscriberPublisher&)
{
  // If we're in lazy subscribe mode, and the first subscriber just
  // connected, then subscribe, #3546
  if(g_lazy && !g_sub)
  {
    ROS_DEBUG("lazy mode; resubscribing");
    subscribe();
  }
}

bool is_latching(const boost::shared_ptr<const ros::M_string>& connection_header)
{
  if (connection_header)
  {
    ros::M_string::const_iterator it = connection_header->find("latching");
    if ((it != connection_header->end()) && (it->second == "1"))
    {
      ROS_DEBUG("input topic is latched; latching output topic to match");
      return true;
    }
  }

  return false;
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::set<T>& object)
{
  std::copy(object.begin(), object.end(), std::ostream_iterator<T>(os, " "));
  return os;
}

void wait_for_subscribers()
{
  // Skip if we do not have to wait for any subscriber
  if (g_wait_for_subscribers.empty())
  {
    return;
  }

  // Set the advertise time
  const auto advertise_time = ros::WallTime::now();

  // Separate explicit subscribers
  const auto& num_subscribers = g_wait_for_subscribers.size();

  std::vector<std::string> explicit_subscribers;
  explicit_subscribers.reserve(num_subscribers);
  std::copy_if(g_wait_for_subscribers.begin(), g_wait_for_subscribers.end(), std::back_inserter(explicit_subscribers),
               [](const std::string& subscriber) { return subscriber != "*"; });

  // Wait for the explicit pending subscribers to subscribe
  std::set<std::string> pending_subscribers(explicit_subscribers.begin(), explicit_subscribers.end());
  while (!pending_subscribers.empty() && ros::WallTime::now() < advertise_time + g_wait_for_subscribers_timeout)
  {
    // Get subscribers
    XmlRpc::XmlRpcValue req(ros::this_node::getName()), res, data;
    if (!ros::master::execute("getSystemState", req, res, data, false))
    {
      ROS_ERROR("Failed to communicate with rosmaster");
      return;
    }

    // Check if any of the explicit pending subscribers has subscribed to the output topic advertised
    XmlRpc::XmlRpcValue sub_info = data[1];
    for (int i = 0; i < sub_info.size(); ++i)
    {
      const auto& topic_name = sub_info[i][0];
      if (topic_name != g_output_topic)
      {
        continue;
      }

      // Remove any explicit pending subscribers that has subscribed
      auto& subscribers = sub_info[i][1];
      for (int j = 0; j < subscribers.size(); ++j)
      {
        const auto subscriber_iter = pending_subscribers.find(static_cast<std::string>(subscribers[j]));
        if (subscriber_iter != pending_subscribers.end())
        {
          pending_subscribers.erase(subscriber_iter);
        }
      }
    }

    ROS_DEBUG_STREAM("Waiting for explicit subscribers [" << pending_subscribers << "] to " << g_output_topic);
    ros::WallDuration(0.1).sleep();
  }

  // Check if we timed out
  if (!pending_subscribers.empty())
  {
    ROS_WARN_STREAM("The subscribers [" << pending_subscribers << "] did not connect after waiting longer than "
                                       << g_wait_for_subscribers_timeout);
    return;
  }

  // Wait for one more subscriber with any name, if requested
  while (g_pub.getNumSubscribers() < num_subscribers &&
         ros::WallTime::now() < advertise_time + g_wait_for_subscribers_timeout)
  {
    ROS_DEBUG_STREAM("Waiting for subscribers to " << g_output_topic);
    ros::WallDuration(0.1).sleep();
  }

  // Check if we timed out
  if (g_pub.getNumSubscribers() < num_subscribers)
  {
    ROS_WARN_STREAM("No (other) subscriber connected after waiting longer than " << g_wait_for_subscribers_timeout);
  }

  // Wait a bit more so the subscribers are connected properly
  if (g_pub.getNumSubscribers() > 0)
  {
    ros::WallDuration(g_wait_for_subscribers_connect_time).sleep();
  }
}

void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event)
{
  boost::shared_ptr<ShapeShifter const> const &msg = msg_event.getConstMessage();
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();

  if (!g_advertised)
  {
    const bool latch = g_force_latch ? g_force_latch_value : is_latching(connection_header);
    g_pub = msg->advertise(*g_node, g_output_topic, 10, latch, conn_cb);
    g_advertised = true;
    printf("advertised as %s\n", g_output_topic.c_str());

    // Allow subscribers to connect to the advertise topic, so they can received the first message published
    wait_for_subscribers();
  }
  // If we're in lazy subscribe mode, and nobody's listening, 
  // then unsubscribe, #3546.
  if(g_lazy && !g_pub.getNumSubscribers())
  {
    ROS_DEBUG("lazy mode; unsubscribing");
    delete g_sub;
    g_sub = NULL;
  }
  else
  {
    if(g_use_messages)
    {
      ros::Time now;
      if(g_use_wallclock)
        now.fromSec(ros::WallTime::now().toSec());
      else
        now = ros::Time::now();
      if (g_last_time > now)
      {
        ROS_WARN("Detected jump back in time, resetting throttle period to now for.");
        g_last_time = now;
      }
      if((now - g_last_time) > g_period)
      {
        g_pub.publish(msg);
        g_last_time = now;
      }
    }
    else
    {
      // pop the front of the queue until it's within the window
      ros::Time now;
      if(g_use_wallclock)
        now.fromSec(ros::WallTime::now().toSec());
      else
        now = ros::Time::now();
      const double t = now.toSec();
      while (!g_sent.empty() && g_sent.front().t < t - g_window)
        g_sent.pop_front();
      // sum up how many bytes are in the window
      uint32_t bytes = 0;
      for (deque<Sent>::iterator i = g_sent.begin(); i != g_sent.end(); ++i)
        bytes += i->len;
      if (bytes < g_bps)
      {
        g_pub.publish(msg);
        g_sent.push_back(Sent(t, msg->size()));
      }
    }
  }
}

#define USAGE "\nusage: \n"\
           "  throttle messages IN_TOPIC MSGS_PER_SEC [OUT_TOPIC]]\n"\
           "OR\n"\
           "  throttle bytes IN_TOPIC BYTES_PER_SEC WINDOW [OUT_TOPIC]]\n\n"\
           "  This program will drop messages from IN_TOPIC so that either: the \n"\
           "  average bytes per second on OUT_TOPIC, averaged over WINDOW \n"\
           "  seconds, remains below BYTES_PER_SEC, or: the minimum inter-message\n"\
           "  period is 1/MSGS_PER_SEC. The messages are output \n"\
           "  to OUT_TOPIC, or (if not supplied), to IN_TOPIC_throttle.\n\n"

int main(int argc, char **argv)
{
  if(argc < 3)
  {
    puts(USAGE);
    return 1;
  }

  g_input_topic = string(argv[2]);

  std::string topic_name;
  if(!getBaseName(string(argv[2]), topic_name))
    return 1;

  ros::init(argc, argv, topic_name + string("_throttle"),
            ros::init_options::AnonymousName);
  bool unreliable = false;
  ros::NodeHandle pnh("~");
  pnh.getParam("wall_clock", g_use_wallclock);
  pnh.getParam("unreliable", unreliable);
  pnh.getParam("lazy", g_lazy);
  g_force_latch = pnh.getParam("force_latch", g_force_latch_value);

  double wait_for_subscribers_timeout{ g_wait_for_subscribers_timeout.toSec() };
  pnh.getParam("wait_for_subscribers_timeout", wait_for_subscribers_timeout);
  g_wait_for_subscribers_timeout.fromSec(wait_for_subscribers_timeout);

  double wait_for_subscribers_connect_time{ g_wait_for_subscribers_connect_time.toSec() };
  pnh.getParam("wait_for_subscribers_connect_time", wait_for_subscribers_connect_time);
  g_wait_for_subscribers_connect_time.fromSec(wait_for_subscribers_connect_time);

  pnh.getParam("wait_for_subscribers", g_wait_for_subscribers);

  // Remove duplicates
  std::sort(g_wait_for_subscribers.begin(), g_wait_for_subscribers.end());
  g_wait_for_subscribers.erase(std::unique(g_wait_for_subscribers.begin(), g_wait_for_subscribers.end()),
                               g_wait_for_subscribers.end());

  if (unreliable)
    g_th.unreliable().reliable(); // Prefers unreliable, but will accept reliable.

  if(!strcmp(argv[1], "messages"))
    g_use_messages = true;
  else if(!strcmp(argv[1], "bytes"))
    g_use_messages = false;
  else
  {
    puts(USAGE);
    return 1;
  }

  if(g_use_messages && argc == 5)
    g_output_topic = string(argv[4]);
  else if(!g_use_messages && argc == 6)
    g_output_topic = string(argv[5]);
  else
    g_output_topic = g_input_topic + "_throttle";

  if(g_use_messages)
  {
    if(argc < 4)
    {
      puts(USAGE);
      return 1;
    }
    g_period = ros::Duration(1.0/atof(argv[3]));
  }
  else
  {
    if(argc < 5)
    {
      puts(USAGE);
      return 1;
    }
    g_bps = atoi(argv[3]);
    g_window = atof(argv[4]);
  }

  ros::NodeHandle n;
  g_node = &n;
  subscribe();
  ros::spin();
  return 0;
}

