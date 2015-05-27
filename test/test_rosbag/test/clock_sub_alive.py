#!/usr/bin/env python

import sys
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


class ClockSubAlive(unittest.TestCase):

    def msg_cb(self, msg):
        self.time = time.time()

    def clock_cb(self, clock):
        self.success = True

    def test_clock_sub_alive(self):
        rospy.init_node('clock_sub_alive')

        self.success = False

        self.time = time.time()
        rospy.Subscriber("blabla", String, self.msg_cb)
        rospy.Subscriber("clock", Clock, self.clock_cb)

        # wait 5 sec after the last message has been received, to check clock
        while (time.time() - self.time) <= 5:
            time.sleep(1)

        self.success = False
        time.sleep(2)

        self.assertTrue(self.success)

if __name__ == '__main__':
    rostest.rosrun('rosbag', 'clock_sub_alive', ClockSubAlive, sys.argv)
