#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

import rospy
import unittest

from message_filters import Cache, Subscriber
import unittest

from std_msgs.msg import String

PKG='message_filters'

class AnonymMsg:
    class AnonymHeader:
        stamp = None;
        def __init__(self):
            self.stamp = rospy.Time()

    header = None
    def __init__(self):
        self.header = AnonymMsg.AnonymHeader()

class TestCache(unittest.TestCase):

    def test_all_funcs(self):
        sub = Subscriber("/empty", String) 
        cache = Cache(sub, 5)
        
        msg = AnonymMsg(); 
        msg.header.stamp = rospy.Time(0)
        cache.add(msg)
        
        msg = AnonymMsg()
        msg.header.stamp = rospy.Time(1)
        cache.add(msg)
        
        msg = AnonymMsg()
        msg.header.stamp = rospy.Time(2)
        cache.add(msg)
        
        msg = AnonymMsg()
        msg.header.stamp = rospy.Time(3)
        cache.add(msg)
        
        msg = AnonymMsg()
        msg.header.stamp = rospy.Time(4)
        cache.add(msg)
        
        l = len(cache.getInterval(rospy.Time(2.5),
                                  rospy.Time(3.5)))
        self.assertEquals(l,1, "invalid number of messages" +
                               " returned in getInterval 1")
        
        l = len(cache.getInterval(rospy.Time(2),
                                  rospy.Time(3)))
        self.assertEquals(l,2, "invalid number of messages" +
                               " returned in getInterval 2")
        
        l = len(cache.getInterval(rospy.Time(0),
                                  rospy.Time(4)))
        self.assertEquals(l,5, "invalid number of messages" +
                               " returned in getInterval 3")
        
        s = cache.getElemAfterTime(rospy.Time(0)).header.stamp;
        self.assertEqual(s, rospy.Time(0),
                         "invalid msg return by getElemAfterTime")
        
        s = cache.getElemBeforeTime(rospy.Time(3.5)).header.stamp;
        self.assertEqual(s, rospy.Time(3),
                         "invalid msg return by getElemBeforeTime")
        
        s = cache.getLastestTime();
        self.assertEqual(s, rospy.Time(4),
                         "invalid stamp return by getLastestTime")
        
        s = cache.getOldestTime();
        self.assertEqual(s, rospy.Time(0),
                         "invalid stamp return by getOldestTime")
        
        # Add another msg to fill the buffer 
        msg = AnonymMsg()
        msg.header.stamp = rospy.Time(5)
        cache.add(msg)
        
        # Test that it discarded the right one
        s = cache.getOldestTime();
        self.assertEqual(s, rospy.Time(1),
                         "wrong message discarded")
        
if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_message_filters_cache', TestCache)
    #TestCache().test_all_funcs()

