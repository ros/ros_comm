/*
 * Copyright (C) 2017, SRI International
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"

std::string counter_topic( "/counter" );
std::string chatter_topic( "/chatter" );

void counter_cb( const std_msgs::Int64::ConstPtr& msg )
{
  static int count( 0 );
  if ( count++ % 10 == 1 ) {
    ROS_INFO( "%d. %s -> %ld", count, counter_topic.c_str(), msg->data );
  }
}

void chatter_cb( const std_msgs::String::ConstPtr& msg )
{
  static int count( 0 );
  if ( count++ % 10 == 1 ) {
    ROS_INFO( "%d. %s -> %s", count, counter_topic.c_str(), msg->data.c_str() );
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  loop_rate.sleep();
  std::cout << "Subscribing to " << chatter_topic << std::endl;
  ros::Subscriber chatter_sub = n.subscribe( chatter_topic, 10, chatter_cb );

  loop_rate.sleep();
  std::cout << "Subscribing to " << counter_topic << std::endl;
  ros::Subscriber counter_sub = n.subscribe( counter_topic, 10, counter_cb );

  loop_rate.sleep();
  ros::spin();

  return 0;
}
