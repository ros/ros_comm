#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
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

import os
import sys
import tempfile
import unittest
import rospy
from rosbag import Bag
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rosbag_msgs.msg import SnapshotStatus
from rosbag_msgs.srv import TriggerSnapshot, TriggerSnapshotRequest, TriggerSnapshotResponse


class TestRosbagSnapshot(unittest.TestCase):
    '''
    Tests the "rosbag snapshot" command.
    Relies on the nodes launched in snapshot.test
    '''
    def __init__(self, *args):
        self.params = rospy.get_param("snapshot")
        self._parse_params(self.params)
        self.last_status = None
        self.status_sub = rospy.Subscriber("snapshot_status", SnapshotStatus, self._status_cb, queue_size=5)
        self.trigger = rospy.ServiceProxy("trigger_snapshot", TriggerSnapshot)
        self.enable = rospy.ServiceProxy("enable_snapshot", SetBool)
        super(TestRosbagSnapshot, self).__init__(*args)

    def _parse_params(self, params):
        '''
        Parse launch parameters of snapshot to cache the topic limits in a map
        so it is easier to check compliance in later tests.
        '''
        self.topic_limits = {}
        self.default_duration_limit = params['default_duration_limit']
        self.default_memory_limit = params['default_memory_limit']
        for topic_obj in self.params['topics']:
            duration = self.default_duration_limit
            memory = self.default_memory_limit
            if type(topic_obj) == dict:
                topic = topic_obj.keys()[0]
                duration = topic_obj[topic].get('duration', duration)
                memory = topic_obj[topic].get('memory', memory)
            else:
                topic = topic_obj
            topic = rospy.resolve_name(topic)
            duration = rospy.Duration(duration)
            memory = 1E6 * memory
            self.topic_limits[topic] = (duration, memory)

    def _status_cb(self, msg):
        self.last_status = msg

    def _assert_no_data(self, topics=[]):
        '''
        Asserts that calling TriggerWrite service with
        specifed parameters responds non-success and did not create
        a bag file.
        '''
        filename = tempfile.mktemp()
        res = self.trigger(filename=filename, topics=topics)
        self.assertFalse(res.success)
        self.assertEqual(res.message, TriggerSnapshotResponse.NO_DATA)
        self.assertFalse(os.path.isfile(filename))

    def _assert_record_success(self, data):
        '''
        Assert that the recording SetBool service responds with success
        '''
        res = self.enable(data)
        self.assertTrue(res.success)
        self.assertEqual(res.message, "")

    def _pause(self):
        self._assert_record_success(False)

    def _resume(self):
        self._assert_record_success(True)

    def _assert_write_success(self, topics=[], prefix_mode=False, **kwargs):
        '''
        Asserts that the TriggerWrite services succeeds for the specified request arguments
        and that the specified bag file is actually created
        @param prefix_mode: If True, don't put .bag at the end of reqest to check prefix filename mode
        '''
        if prefix_mode:
            d = tempfile.mkdtemp()
            filename = tempfile.mktemp(dir=d)
        else:
            filename = tempfile.mktemp(suffix='.bag')
        req = TriggerSnapshotRequest(filename=filename, topics=topics, **kwargs)
        res = self.trigger(req)
        self.assertTrue(res.success, msg="snapshot should have succeeded. message: {}".format(res.message))
        self.assertTrue(res.message == "")
        if prefix_mode:
            dircontents = os.listdir(d)
            self.assertEqual(len(dircontents), 1)
            filename = os.path.join(d, dircontents[0])
            self.assertEqual(filename[-4:], '.bag')
        self.assertTrue(os.path.isfile(filename))
        return filename

    def _assert_limits_enforced(self, test_topic, duration, memory):
        '''
        Asserts that the measured duration and memory for a topic comply with the launch parameters
        @param topic: string
        @param duration: rospy.Duration, age of buffer
        @param memory: integer, bytes of memory used
        '''
        test_topic = rospy.resolve_name(test_topic)
        self.assertIn(test_topic, self.topic_limits)
        limits = self.topic_limits[test_topic]
        if limits[0] > rospy.Duration():
            self.assertLessEqual(duration, limits[0])
        if limits[1] > 0:
            self.assertLessEqual(memory, limits[1])

    def _assert_status_valid(self):
        '''
        Check that a status message contains info on all subscribed topics
        and reports that their buffer complies with the configured limits.
        '''
        self.assertIsNotNone(self.last_status)  # A message was recieved
        topics = [msg.topic for msg in self.last_status.topics]
        # Oneliners :)
        status_topics = [rospy.resolve_name(topic.keys()[0] if type(topic) == dict else topic)
                         for topic in self.params['topics']]
        self.assertEquals(set(topics), set(status_topics))  # Topics from params are same as topics in status message
        for topic in self.last_status.topics:
            duration = topic.window_stop - topic.window_start
            memory = topic.traffic
            self._assert_limits_enforced(topic.topic, duration, memory)

    def _assert_bag_valid(self, filename, topics=None, start_time=None, stop_time=None):
        '''
        Open the bagfile at the specified filename and read it to ensure topic limits were
        enforced and the optional topic list and start/stop times are also enforced.
        '''
        bag = Bag(filename)
        topics_dict = bag.get_type_and_topic_info()[1]
        bag_topics = set(topics_dict.keys())
        param_topics = set(self.topic_limits.keys())
        if topics:
            self.assertEqual(bag_topics, set(topics))
        self.assertTrue(bag_topics.issubset(param_topics))
        for topic in topics_dict:
            size = topics_dict[topic].message_count * 8  # Calculate stored message size as each message is 8 bytes
            gen = bag.read_messages(topics=topic)
            _, _, first_time = gen.next()
            if start_time:
                self.assertGreaterEqual(first_time, start_time)
            for _, _, last_time in gen:  # Read through all messages so last_time is valid
                pass
            if stop_time:
                self.assertLessEqual(last_time, stop_time)
            duration = last_time - first_time
            self._assert_limits_enforced(topic, duration, size)

    def test_1_service_connects(self):
        '''
        Check that both services provided by snapshot exist.
        '''
        self.trigger.wait_for_service()
        self.enable.wait_for_service()

    def test_write_all(self):
        '''
        Wait long enough for memory & duration limits to need to be used
        '''
        rospy.sleep(3.0)  # Give some time to fill buffers to maximums
        self._assert_status_valid()
        filename = self._assert_write_success(prefix_mode=True)
        self._assert_bag_valid(filename)

    def test_write_advanced(self):
        '''
        Test the more advanced features: pausing and resuming, specific write times, and specific topic list.
        '''
        # Pause, resume, and pause again so buffer should only contain data from a known time internal
        self._pause()
        rospy.sleep(1.5)
        start = rospy.Time.now()
        self._resume()
        rospy.sleep(3.0)
        self._pause()
        stop = rospy.Time.now()
        rospy.sleep(1.0)

        # Write all buffer data, check that only data from resumed interval is present
        filename = self._assert_write_success()
        self._assert_bag_valid(filename, start_time=start, stop_time=stop)

        # With recording still paused (same buffer as last write), write only a shorter time range
        cropped_start = start + rospy.Duration(0.5)
        cropped_stop = stop - rospy.Duration(1.0)
        filename = self._assert_write_success(start_time=cropped_start, stop_time=cropped_stop)
        self._assert_bag_valid(filename, start_time=cropped_start, stop_time=cropped_stop)

        # Write only specific topics and ensure that only those are present in bag file
        specific_topics = [rospy.resolve_name(topic) for topic in ['/test2', 'test1']]
        filename = self._assert_write_success(topics=specific_topics)
        self._assert_bag_valid(filename, topics=specific_topics)

        # Resume recording for other tests
        self._resume()

    def test_invalid_topics(self):
        '''
        Test that an invalid topic or one not subscribed to fails
        '''
        self._assert_no_data(['_invalid_graph_name'])
        self._assert_no_data(['/test4'])


if __name__ == '__main__':
    import rostest
    PKG = 'rosbag'
    rospy.init_node('test_rosbag_snapshot', anonymous=True)
    rostest.run(PKG, 'TestRosbagSnapshot', TestRosbagSnapshot, sys.argv)
