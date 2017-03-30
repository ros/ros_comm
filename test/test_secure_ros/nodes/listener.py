#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


import rospy
from std_msgs.msg import String, Int64
import time

chatter_count = 0
chatter_topic = "/chatter"
counter_count = 0
counter_topic = "/counter"


def chatter_callback(msg):
  global chatter_count
  global chatter_topic
  chatter_count += 1
  if chatter_count % 10 == 1:
    rospy.loginfo( "%d. %s -> %s" % ( chatter_count, chatter_topic, msg.data ) )


def counter_callback(msg):
  global counter_count
  global counter_topic
  counter_count += 1
  if counter_count % 10 == 1:
    rospy.loginfo( "%d. %s -> %s" % ( counter_count, counter_topic, msg.data ) )

  
def listener( name = "listener", anon = False ):
  rospy.init_node( name, anonymous = anon )
  time.sleep(0.5)
  print( "Started node %s" % rospy.get_name() )
  global chatter_topic
  global counter_topic
  print( "Subscribing to %s" % ( chatter_topic ) )
  chatter_sub = rospy.Subscriber("chatter", String, chatter_callback)
  time.sleep(0.5)
  print( "Subscribing to %s" % ( counter_topic ) )
  counter_sub = rospy.Subscriber("counter", Int64, counter_callback)

  rate = rospy.Rate(10.) # 1hz
  while not rospy.is_shutdown():
      rate.sleep()
  counter_sub.unregister()
  chatter_sub.unregister()

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument( "--name", type = str, default = "listener", help="Node name" )
  parser.add_argument( "--anon", action = "store_true", help="Anonymize node name" )
  args = parser.parse_args()

  listener( name = args.name, anon = args.anon )
  time.sleep(0.5)
