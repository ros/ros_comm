///////////////////////////////////////////////////////////////////////////////
// demux is a generic ROS topic demultiplexer: one input topic is fanned out
// to 1 of N output topics. A service is provided to select between the outputs
//
// Copyright (C) 2014, Andreas Hermann
// Code copied and adapted from the "toppic mux" by Morgan Quigley
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


#include <cstdio>
#include <vector>
#include <list>
#include "ros/console.h"
#include "std_msgs/String.h"
#include "topic_tools/DemuxSelect.h"
#include "topic_tools/DemuxAdd.h"
#include "topic_tools/DemuxList.h"
#include "topic_tools/DemuxDelete.h"
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using std::list;
using namespace topic_tools;

const static string g_none_topic = "__none";

static ros::NodeHandle *g_node = NULL;

static string g_input_topic;
static ros::Subscriber g_sub; // the input toppic
static ros::Publisher g_pub_selected; // publishes name of selected publisher toppic

struct pub_info_t
{
  std::string topic_name;
  ros::Publisher *pub;
};

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg);

static list<struct pub_info_t> g_pubs; // the list of publishers
static list<struct pub_info_t>::iterator g_selected = g_pubs.end();

bool sel_srv_cb( topic_tools::DemuxSelect::Request  &req,
                 topic_tools::DemuxSelect::Response &res )
{
  bool ret = false;
  if (g_selected != g_pubs.end()) {
    res.prev_topic = g_selected->topic_name;

    // Unregister old topic
    ROS_INFO("Unregistering %s", res.prev_topic.c_str());
    if (g_selected->pub)
	  g_selected->pub->shutdown();
    delete g_selected->pub;
    g_selected->pub = NULL;
  }
  else
    res.prev_topic = string("");

  // see if it's the magical '__none' topic, in which case we open the circuit
  if (req.topic == g_none_topic)
  {
    ROS_INFO("demux selected to no output.");

    g_selected = g_pubs.end();
    ret = true;
  }
  else
  {
    ROS_INFO("trying to switch demux to %s", req.topic.c_str());
    // spin through our vector of inputs and find this guy
    for (list<struct pub_info_t>::iterator it = g_pubs.begin();
	 it != g_pubs.end();
	 ++it)
    {
      if (ros::names::resolve(it->topic_name) == ros::names::resolve(req.topic))
      {
        g_selected = it;
        ROS_INFO("demux selected output: [%s]", it->topic_name.c_str());
        ret = true;
      }
    }
    if(!ret)
    {
    	ROS_WARN("Failed to switch to non-existing topic %s in demux.", req.topic.c_str());
    }
  }

  if(ret)
  {
    std_msgs::String t;
    t.data = req.topic;
    g_pub_selected.publish(t);
  }

  return ret;
}

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg)
{
  ROS_INFO("Received an incoming msg ...");
  // when a message is incoming, check, if the requested publisher is already existing.
  // if not, create it with the information available from the incoming message.
  if(!g_selected->pub)
  {
	  try
	  {
		  g_selected->pub = new ros::Publisher(msg->advertise(*g_node, g_selected->topic_name, 10, false));
	  }
	  catch(ros::InvalidNameException& e)
	  {
		ROS_WARN("failed to add topic %s to demux, because it's an invalid name: %s",
				g_selected->topic_name.c_str(), e.what());
		return;
	  }

	  ROS_INFO("Added publisher %s to demux! Sleeping for 0.5 secs.", g_selected->topic_name.c_str());
	  ros::Duration(0.5).sleep(); // sleep for half a second
  }

  // finally: send out the message over the active publisher
  g_selected->pub->publish(msg);
  ROS_INFO("... and sent it out again!");
}

bool list_topic_cb(topic_tools::DemuxList::Request& req,
	 	   topic_tools::DemuxList::Response& res)
{
  for (list<struct pub_info_t>::iterator it = g_pubs.begin();
       it != g_pubs.end();
       ++it)
  {
    res.topics.push_back(it->topic_name);
  }

  return true;
}

bool add_topic_cb(topic_tools::DemuxAdd::Request& req,
		  topic_tools::DemuxAdd::Response& res)
{
  // Check that it's not already in our list
  ROS_INFO("trying to add %s to demux", req.topic.c_str());

  // Can't add the __none topic
  if(req.topic == g_none_topic)
  {
    ROS_WARN("failed to add topic %s to demux, because it's reserved for special use",
	     req.topic.c_str());
    return false;
  }

  // spin through our vector of inputs and find this guy
  for (list<struct pub_info_t>::iterator it = g_pubs.begin();
       it != g_pubs.end();
       ++it)
  {
    if (ros::names::resolve(it->topic_name) == ros::names::resolve(req.topic))
    {
      ROS_WARN("tried to add a topic that demux was already publishing: [%s]",
	       it->topic_name.c_str());
      return false;
    }
  }

  struct pub_info_t pub_info;
  pub_info.topic_name = ros::names::resolve(req.topic);
  pub_info.pub = NULL;
  g_pubs.push_back(pub_info);

  ROS_INFO("PRE added %s to demux", req.topic.c_str());

  return true;
}

bool del_topic_cb(topic_tools::DemuxDelete::Request& req,
		  topic_tools::DemuxDelete::Response& res)
{
  // Check that it's in our list
  ROS_INFO("trying to delete %s from demux", req.topic.c_str());
  // spin through our vector of inputs and find this guy
  for (list<struct pub_info_t>::iterator it = g_pubs.begin();
       it != g_pubs.end();
       ++it)
  {
    if (ros::names::resolve(it->topic_name) == ros::names::resolve(req.topic))
    {
      // Can't delete the currently selected input, #2863
      if(it == g_selected)
      {
        ROS_WARN("tried to delete currently selected topic %s from demux", req.topic.c_str());
        return false;
      }
      if (it->pub)
        it->pub->shutdown();
      delete it->pub;
      g_pubs.erase(it);
      ROS_INFO("deleted topic %s from demux", req.topic.c_str());
      return true;
    }
  }

  ROS_WARN("tried to delete non-published topic %s from demux", req.topic.c_str());
  return false;
}

int main(int argc, char **argv)
{
  vector<string> args;
  ros::removeROSArgs(argc, (const char**)argv, args);

  if (args.size() < 3)
  {
    printf("\nusage: demux IN_TOPIC OUT_TOPIC1 [OUT_TOPIC2 [...]]\n\n");
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(args[1], topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_demux"),
            ros::init_options::AnonymousName);
  vector<string> topics;
  for (unsigned int i = 2; i < args.size(); i++)
    topics.push_back(args[i]);
  ros::NodeHandle n;
  g_node = &n;
  g_input_topic = args[1];
  // Put our API into the "demux" namespace, which the user should usually remap
  ros::NodeHandle pnh("~");

  // Latched publisher for selected output topic name
  g_pub_selected = pnh.advertise<std_msgs::String>(string("selected"), 1, true);

  for (size_t i = 0; i < topics.size(); i++)
  {
    struct pub_info_t pub_info;
    pub_info.topic_name = ros::names::resolve(topics[i]);
    pub_info.pub = NULL;
    g_pubs.push_back(pub_info);
    ROS_INFO("PRE added %s to demux", topics[i].c_str());
  }
  g_selected = g_pubs.begin(); // select first topic to start
  std_msgs::String t;
  t.data = g_selected->topic_name;
  g_pub_selected.publish(t);

  // Create the one subscriber
  g_sub = ros::Subscriber(g_node->subscribe<ShapeShifter>(g_input_topic, 10, boost::bind(in_cb, _1)));


  // New service
  ros::ServiceServer ss_select = pnh.advertiseService(string("select"), sel_srv_cb);
  ros::ServiceServer ss_add = pnh.advertiseService(string("add"), add_topic_cb);
  ros::ServiceServer ss_list = pnh.advertiseService(string("list"), list_topic_cb);
  ros::ServiceServer ss_del = pnh.advertiseService(string("delete"), del_topic_cb);

  // Run
  ros::spin();

  // Destruction
  for (list<struct pub_info_t>::iterator it = g_pubs.begin();
       it != g_pubs.end();
       ++it)
  {
    if (it->pub)
      it->pub->shutdown();
    delete it->pub;
  }

  g_pubs.clear();
  return 0;
}

