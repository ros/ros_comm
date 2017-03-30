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

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  loop_rate.sleep();

  std::string chatter_topic( "/chatter" );
  std::cout << "Publishing to " << chatter_topic << std::endl;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>( chatter_topic, 10);

  loop_rate.sleep();

  std::string counter_topic( "/counter" );
  std::cout << "Publishing to " << counter_topic << std::endl;
  ros::Publisher counter_pub = n.advertise<std_msgs::Int64>( counter_topic, 10);

  loop_rate.sleep();

  int count = 0;
  std_msgs::String chatter_msg;
  std_msgs::Int64 counter_msg;
  while ( ros::ok() ) {
    std::stringstream ss;
    ss << "hello world " << count;
    chatter_msg.data = ss.str();
    counter_msg.data = count;

    if ( count % 10 == 0 ) {
      ROS_INFO( "%d. %s <- %s", count, chatter_topic.c_str(), chatter_msg.data.c_str() );
    }
    chatter_pub.publish(chatter_msg);

    loop_rate.sleep();

    if ( count % 10 == 0 ) {
      ROS_INFO( "%d. %s <- %ld", count, counter_topic.c_str(), counter_msg.data );
    }
    counter_pub.publish(counter_msg);

    loop_rate.sleep();

    ros::spinOnce();
    ++count;
  }

  return 0;
}
