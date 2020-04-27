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

    callback_data = None

    @classmethod
    def _set_callback_data(cls, data):
        cls.callback_data = data

    @classmethod
    def _subscriber_callback(cls, data):
        if MSG_PREFIX in data.msg:
            cls._set_callback_data(data)

    @classmethod
    def setUpClass(cls):
        cls.subscriber = rospy.Subscriber(SUBTOPIC, Log, cls._subscriber_callback)
        # https://answers.ros.org/question/251194/rospy-subscriber-needs-sleep-some-time-until-the-first-message-is-received/
        rospy.sleep(0.5)

    def setUp(self):
        self._set_callback_data(None)

    # Test that rosout is outputting debug messages as expected
    def test_rosout_dbg(self):
        self.assertIsNone(self.callback_data, 'invalid test fixture')

        log_msg = MSG_PREFIX + 'logging test debug message'
        rospy.logdebug(log_msg)

        # wait for log messages to be received
        timeout_time = time.time() + TIMEOUT
        while not self.callback_data and time.time() < timeout_time:
            time.sleep(0.1)
        print(self.callback_data)

        # checking number of messages received on /rosout is expected
        self.assertIsNotNone(self.callback_data, 'did not receive expected message')

        # checking contents of message
        self.assertEquals(rospy.DEBUG, self.callback_data.level)
        self.assertEquals(SUBTOPIC, self.callback_data.name)
        self.assertEquals(log_msg, self.callback_data.msg)
        self.assertEquals(NAME+'.py', self.callback_data.file)
        self.assertEquals('TestRosout.test_rosout_dbg', self.callback_data.function)
        self.assertEquals([SUBTOPIC], self.callback_data.topics)

    # Test that rosout is outputting info messages as expected
    def test_rosout_info(self):
        self.assertIsNone(self.callback_data, 'invalid test fixture')

        log_msg = MSG_PREFIX + 'logging test info message'
        rospy.loginfo(log_msg)

        # wait for log messages to be received
        timeout_time = time.time() + TIMEOUT
        while not self.callback_data and time.time() < timeout_time:
            time.sleep(0.1)
        print(self.callback_data)

        # checking number of messages received on /rosout is expected
        self.assertIsNotNone(self.callback_data, 'did not receive expected message')

        # checking contents of message
        self.assertEquals(rospy.INFO, self.callback_data.level)
        self.assertEquals(SUBTOPIC, self.callback_data.name)
        self.assertEquals(log_msg, self.callback_data.msg)
        self.assertEquals(NAME+'.py', self.callback_data.file)
        self.assertEquals('TestRosout.test_rosout_info', self.callback_data.function)
        self.assertEquals([SUBTOPIC], self.callback_data.topics)

    # Test that rosout is outputting warning messages as expected
    def test_rosout_warn(self):
        self.assertIsNone(self.callback_data, 'invalid test fixture')

        log_msg = MSG_PREFIX + 'logging test warning message'
        rospy.logwarn(log_msg)

        # wait for log messages to be received
        timeout_time = time.time() + TIMEOUT
        while not self.callback_data and time.time() < timeout_time:
            time.sleep(0.1)
        print(self.callback_data)

        # checking number of messages received on /rosout is expected
        self.assertIsNotNone(self.callback_data, 'did not receive expected message')

        # checking contents of message
        self.assertEquals(rospy.WARN, self.callback_data.level)
        self.assertEquals(SUBTOPIC, self.callback_data.name)
        self.assertEquals(log_msg, self.callback_data.msg)
        self.assertEquals(NAME+'.py', self.callback_data.file)
        self.assertEquals('TestRosout.test_rosout_warn', self.callback_data.function)
        self.assertEquals([SUBTOPIC], self.callback_data.topics)

    # Test that rosout is outputting error messages as expected
    def test_rosout_err(self):
        self.assertIsNone(self.callback_data, 'invalid test fixture')

        log_msg = MSG_PREFIX + 'logging test error message'
        rospy.logerr(log_msg)

        # wait for log messages to be received
        timeout_time = time.time() + TIMEOUT
        while not self.callback_data and time.time() < timeout_time:
            time.sleep(0.1)
        print(self.callback_data)

        # checking number of messages received on /rosout is expected
        self.assertIsNotNone(self.callback_data, 'did not receive expected message')

        # checking contents of message
        self.assertEquals(rospy.ERROR, self.callback_data.level)
        self.assertEquals(SUBTOPIC, self.callback_data.name)
        self.assertEquals(log_msg, self.callback_data.msg)
        self.assertEquals(NAME+'.py', self.callback_data.file)
        self.assertEquals('TestRosout.test_rosout_err', self.callback_data.function)
        self.assertEquals([SUBTOPIC], self.callback_data.topics)

    # Test that rosout is outputting fatal messages as expected
    def test_rosout_fatal(self):
        self.assertIsNone(self.callback_data, 'invalid test fixture')

        log_msg = MSG_PREFIX + 'logging test fatal message'
        rospy.logfatal(log_msg)

        # wait for log messages to be received
        timeout_time = time.time() + TIMEOUT
        while not self.callback_data and time.time() < timeout_time:
            time.sleep(0.1)
        print(self.callback_data)

        # checking number of messages received on /rosout is expected
        self.assertIsNotNone(self.callback_data, 'did not receive expected message')

        # checking contents of message
        self.assertEquals(rospy.FATAL, self.callback_data.level)
        self.assertEquals(SUBTOPIC, self.callback_data.name)
        self.assertEquals(log_msg, self.callback_data.msg)
        self.assertEquals(NAME+'.py', self.callback_data.file)
        self.assertEquals('TestRosout.test_rosout_fatal', self.callback_data.function)
        self.assertEquals([SUBTOPIC], self.callback_data.topics)


if __name__ == '__main__':
    rospy.init_node(NAME, log_level=rospy.DEBUG)
    rostest.run(PKG, NAME, TestRosout, sys.argv)
