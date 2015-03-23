#!/usr/bin/env python

import sys
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


class ClockSub(unittest.TestCase):

    def msg_cb(self, msg):
        pass

    def clock_cb(self, clock):
        self.success = True

    def test_clock_sub(self):
        rospy.init_node('clock_sub')

        self.success = False

        rospy.Subscriber("blabla", String, self.msg_cb)
        rospy.Subscriber("clock", Clock, self.clock_cb)

        time.sleep(2)

        self.assertEqual(self.success, True)

if __name__ == '__main__':
    rostest.rosrun('rosbag', 'clock_sub', ClockSub, sys.argv)
