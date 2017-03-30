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

def talker( name = "talker", anon = False ):
  rospy.init_node( name, anonymous = anon )
  time.sleep(0.5)
  print( "Started node %s" % rospy.get_name() )
  chatter_topic = "/chatter"
  counter_topic = "/counter"
  print( "Publishing to %s" % chatter_topic )
  pub1 = rospy.Publisher( chatter_topic, String, queue_size=10)
  time.sleep(0.5)
  print( "Publishing to %s" % counter_topic )
  pub2 = rospy.Publisher( counter_topic, Int64, queue_size=10)
  cnt = 0
  rate = rospy.Rate(1.) # 1hz
  while not rospy.is_shutdown():
    chatter_msg = "hello world %d" % ( cnt )
    counter_msg = Int64( data = cnt )
    if cnt % 10 == 1:
      rospy.loginfo( "%d. %s <- %s" % ( cnt, chatter_topic, chatter_msg ) )
    pub1.publish( chatter_msg )
    rate.sleep()
    if cnt % 10 == 1:
      rospy.loginfo( "%d. %s <- %s" % ( cnt, counter_topic, counter_msg ) )
    pub2.publish( Int64( data=cnt ) )
    rate.sleep()
    cnt = cnt + 1
  print( "Finished" )

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument( "--name", type = str, default = "talker", help="Node name" )
  parser.add_argument( "--anon", action = "store_true", help="Anonymize node name" )
  args = parser.parse_args()

  try:
    talker( name = args.name, anon = args.anon )
  except rospy.ROSInterruptException:
    pass
  time.sleep(0.5)
