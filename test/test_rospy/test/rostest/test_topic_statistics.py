#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

'''
Integration test for topic statistics
'''

from __future__ import print_function
import sys
import unittest

import rospy
import rostest
from rosgraph_msgs.msg import TopicStatistics

PKG = 'test_rospy'


class TestTopicStatistics(unittest.TestCase):
    def setUp(self):
        self.topic_statistic_msg_map = {}

    def new_msg(self, msg):
        self.topic_statistic_msg_map[msg.topic] = msg

    def assert_eventually(
        self, cond, timeout=rospy.Duration(5.0), interval=rospy.Duration(0.5)
    ):
        started = rospy.Time.now()
        while rospy.Time.now() - started < timeout:
            if cond():
                return True
            rospy.sleep(interval)
        self.assertTrue(False)

    def frequency_acceptable(self, topic, expected, error_margin=0.1):
        ''' return True if topic message's measured frequency
        is within some error margin of expected frequency '''
        msg = self.topic_statistic_msg_map[topic]
        # need at least two messages to compute the period fields
        assert msg.delivered_msgs > 1
        found_freq = 1.0 / msg.period_mean.to_sec()
        rospy.loginfo(
            "Testing {}'s found frequency {} against expected {}".format(
                topic, found_freq, expected))
        return abs(found_freq - expected) / expected <= error_margin

    def test_frequencies(self):
        sub = rospy.Subscriber('/statistics', TopicStatistics, self.new_msg)

        self.assert_eventually(
            lambda: '/very_fast_chatter' in self.topic_statistic_msg_map)
        self.assert_eventually(
            lambda: '/fast_chatter' in self.topic_statistic_msg_map)
        self.assert_eventually(
            lambda: '/slow_chatter' in self.topic_statistic_msg_map)
        self.assert_eventually(
            lambda: '/very_slow_chatter' in self.topic_statistic_msg_map)

        self.assertTrue(self.frequency_acceptable('/very_fast_chatter', 150))
        self.assertTrue(self.frequency_acceptable('/fast_chatter', 53))
        self.assertTrue(self.frequency_acceptable('/slow_chatter', 8))
        self.assertTrue(self.frequency_acceptable('/very_slow_chatter', 0.5))


if __name__ == '__main__':
    rospy.init_node('test_topic_statistics')
    rostest.run(PKG, 'rospy_topic_statistics', TestTopicStatistics, sys.argv)
