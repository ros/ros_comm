#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Amazon.com, Inc. or its affiliates.
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

## Integration test for empty services to test serializers
## and transport

from __future__ import print_function

PKG = 'test_rospy'
NAME = 'test_rosout'

import sys
import time
import unittest

import rospy
import rostest
from rosgraph_msgs.msg import Log

SUBTOPIC = '/rosout'
MSG_PREFIX = '[test_rosout] '
TIMEOUT = 10.0 #seconds


class TestRosout(unittest.TestCase):

    def setUp(self):
        self.callback_data = []

    def _subscriber_callback(self, data):
        if MSG_PREFIX in data.msg:
            self.callback_data.append(data)

    # Test that rosout is outputting as expected by creating a subscriber that subscribes to it
    def test_rosout(self):
        self.assert_(not self.callback_data, 'invalid test fixture')

        rospy.Subscriber(SUBTOPIC, Log, self._subscriber_callback)

        # https://answers.ros.org/question/251194/rospy-subscriber-needs-sleep-some-time-until-the-first-message-is-received/
        rospy.sleep(0.5)

        log_levels = [
            rospy.DEBUG,
            rospy.WARN,
            rospy.INFO,
            rospy.ERROR,
            rospy.FATAL
        ]

        log_msgs = [
            MSG_PREFIX + 'logging test debug message',
            MSG_PREFIX + 'logging test warn message',
            MSG_PREFIX + 'logging test info message',
            MSG_PREFIX + 'logging test err message',
            MSG_PREFIX + 'logging test fatal message'
        ]

        rospy.logdebug(log_msgs[0])
        rospy.logwarn(log_msgs[1])
        rospy.loginfo(log_msgs[2])
        rospy.logerr(log_msgs[3])
        rospy.logfatal(log_msgs[4])

        # wait for log messages to be received
        timeout_time = time.time() + TIMEOUT
        while len(self.callback_data) < len(log_msgs) and time.time() < timeout_time:
            time.sleep(0.1)

        # checking number of messages received on /rosout is expected
        self.assertEquals(len(log_msgs), len(self.callback_data),
            'expected to receive %d log messages but got %d' % (len(log_msgs), len(self.callback_data)))

        # checking contents of messages
        for (callback_data, log_level, log_msg) in zip(self.callback_data, log_levels, log_msgs):
            self.assertEquals(log_level, callback_data.level)
            self.assertEquals(SUBTOPIC, callback_data.name)
            self.assertEquals(log_msg, callback_data.msg)
            self.assertEquals(NAME+'.py', callback_data.file)
            self.assertEquals('TestRosout.test_rosout', callback_data.function)
            self.assertEquals([SUBTOPIC], callback_data.topics)


if __name__ == '__main__':
    rospy.init_node(NAME, log_level=rospy.DEBUG)
    rostest.run(PKG, NAME, TestRosout, sys.argv)
