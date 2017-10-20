#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import unittest
from std_msgs.msg import String


class TestRelayStealth(unittest.TestCase):
    def out_callback(self, msg):
        self.out_msg_count += 1

    def monitor_callback(self, msg):
        self.monitor_msg_count += 1

    def test_stealth_relay(self):
        self.out_msg_count = 0
        self.monitor_msg_count = 0
        sub_out = rospy.Subscriber("/relay_stealth/output", String,
                                   self.out_callback, queue_size=1)
        for i in range(5):
            if sub_out.get_num_connections() == 0:
                rospy.sleep(1)
        self.assertTrue(sub_out.get_num_connections() > 0)

        rospy.sleep(5)
        self.assertEqual(self.out_msg_count, 0)

        sub_monitor = rospy.Subscriber("/original_topic/relay", String,
                                       self.monitor_callback, queue_size=1)
        rospy.sleep(5)
        self.assertGreater(self.monitor_msg_count, 0)
        self.assertGreater(self.out_msg_count, 0)

        cnt = self.out_msg_count
        sub_monitor.unregister()

        rospy.sleep(3)
        self.assertLess(abs(cnt - self.out_msg_count), 30)


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_relay_stealth")
    rostest.rosrun("topic_tools", "test_relay_stealth", TestRelayStealth)
