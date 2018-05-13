#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, JSK Robotics Lab.
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
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import threading
import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
import time
from unittest import TestCase


class ClockPublisher(threading.Thread):
    def __init__(self):
        super(ClockPublisher, self).__init__()
        self.finished = threading.Event()
        self.interval = 0.1
        self.pub_clock = rospy.Publisher("/clock", Clock, queue_size=1)
        self.reset(time.time())

    def run(self):
        while not self.finished.is_set():
            self.finished.wait(self.interval)
            self.pub_clock.publish(self.clock)
            self.clock.clock += rospy.Duration(self.interval)
        self.finished.set()

    def stop(self):
        self.finished.set()

    def reset(self, seconds=0):
        self.clock = Clock(
            clock=rospy.Time.from_seconds(seconds))


class TestThrottleSimtimeLoop(TestCase):
    def setUp(self):
        self.clock_pub = ClockPublisher()
        self.clock_pub.start()
        time.sleep(1)
        self.input_count = 0
        self.throttle_count = 0
        self.sub_throttle = rospy.Subscriber(
            "input_throttle", String, self.callback_throttle, queue_size=1)
        self.sub_input = rospy.Subscriber(
            "input", String, self.callback_input, queue_size=1)

    def tearDown(self):
        self.clock_pub.stop()

    def callback_throttle(self, msg):
        self.throttle_count += 1

    def callback_input(self, msg):
        self.input_count += 1

    def test_throttle_loop(self):
        # wait for throttled message
        for i in range(100):
            if self.throttle_count > 0:
                break
            time.sleep(0.1)
        self.assertGreater(
            self.input_count, 0,
            "Input message comes before rostime moves backward")
        self.assertGreater(
            self.throttle_count, 0,
            "Throttle message comes before rostime moves backward")

        # reset /clock (rostime moves backward)
        self.clock_pub.reset()
        time.sleep(0.1)

        # wait for throttled message
        self.input_count = 0
        self.throttle_count = 0
        for i in range(100):
            if self.throttle_count > 0:
                break
            time.sleep(0.1)
        self.assertGreater(
            self.input_count, 0,
            "Input message comes after rostime moved backward")
        self.assertGreater(
            self.throttle_count, 0,
            "Throttle message comes after rostime moved backward")


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_throttle_simtime_loop")
    rostest.rosrun("topic_tools",
                   "test_throttle_simtime_loop",
                   TestThrottleSimtimeLoop)
