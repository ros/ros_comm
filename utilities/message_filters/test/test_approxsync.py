#!/usr/bin/env python
#
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

import rostest
import rospy
import unittest
import random
import copy

import message_filters
from message_filters import ApproximateTimeSynchronizer

class MockHeader:
    pass

class MockMessage:
    def __init__(self, stamp, data):
        self.header = MockHeader()
        self.header.stamp = stamp
        self.data = data

class MockHeaderlessMessage:
    def __init__(self, data):
        self.data = data

class MockFilter(message_filters.SimpleFilter):
    pass

class TestApproxSync(unittest.TestCase):

    def cb_collector_2msg(self, msg1, msg2):
        self.collector.append((msg1, msg2))

    def test_approx(self):
        rospy.rostime.set_rostime_initialized(True)
        m0 = MockFilter()
        m1 = MockFilter()
        ts = ApproximateTimeSynchronizer([m0, m1], 1, 0.1)
        rospy.rostime.set_rostime_initialized(True)
        ts.registerCallback(self.cb_collector_2msg)

        if 0:
            # Simple case, pairs of messages, make sure that they get combined
            for t in range(10):
                self.collector = []
                msg0 = MockMessage(t, 33)
                msg1 = MockMessage(t, 34)
                m0.signalMessage(msg0)
                self.assertEqual(self.collector, [])
                m1.signalMessage(msg1)
                self.assertEqual(self.collector, [(msg0, msg1)])

        # Scramble sequences of length N.  Make sure that TimeSequencer recombines them.
        random.seed(0)
        for N in range(1, 10):
            m0 = MockFilter()
            m1 = MockFilter()
            seq0 = [MockMessage(rospy.Time(t), random.random()) for t in range(N)]
            seq1 = [MockMessage(rospy.Time(t), random.random()) for t in range(N)]
            # random.shuffle(seq0)
            ts = ApproximateTimeSynchronizer([m0, m1], N, 0.1)
            ts.registerCallback(self.cb_collector_2msg)
            self.collector = []
            for msg in random.sample(seq0, N):
                m0.signalMessage(msg)
            self.assertEqual(self.collector, [])
            for msg in random.sample(seq1, N):
                m1.signalMessage(msg)
            self.assertEqual(set(self.collector), set(zip(seq0, seq1)))

        # Scramble sequences of length N of headerless and header-having messages.
        # Make sure that TimeSequencer recombines them.
        random.seed(0)
        for N in range(1, 10):
            m0 = MockFilter()
            m1 = MockFilter()
            seq0 = [MockMessage(rospy.Time(t), random.random()) for t in range(N)]
            seq1 = [MockHeaderlessMessage(random.random()) for t in range(N)]
            # new shuffled sequences
            seq0r = copy.copy(seq0)
            seq1r = copy.copy(seq1)
            random.shuffle(seq0r)
            random.shuffle(seq1r)
            ts = ApproximateTimeSynchronizer([m0, m1], N, 0.1, allow_headerless=True)
            ts.registerCallback(self.cb_collector_2msg)
            self.collector = []
            for i in range(N):
                # clock time needs to be continuous
                rospy.rostime._set_rostime(rospy.Time(i))
                m0.signalMessage(seq0r[i])
                m1.signalMessage(seq1r[i])
            self.assertEqual(set(self.collector), set(zip(seq0, seq1r)))

        # clear buffer on sequences with non-continuous time
        random.seed(0)
        for N in range(2, 10):
            m0 = MockFilter()
            m1 = MockFilter()
            seq0 = [MockMessage(rospy.Time(t), random.random()) for t in range(N)]
            seq1 = [MockMessage(rospy.Time(t), random.random()) for t in range(N)]
            ts = ApproximateTimeSynchronizer([m0, m1], N, 0.1)
            ts.registerCallback(self.cb_collector_2msg)
            self.collector = []
            seq_time = [rospy.Time(i) for i in range(10, 10+N)]
            # select a random time in sequence at which time jumps backwards
            ind_rand = random.randint(1, N-1)
            random_jump = random.uniform(1/1000.0, 100/1000.0)
            # jump backward in time by 'random_jump', starting at 'ind_rand'
            for i in range(N-1, ind_rand-1, -1):
                seq_time[i] = seq_time[i-1]-rospy.Duration(random_jump)
            for i in range(N):
                rospy.rostime._set_rostime(seq_time[i])
                m0.signalMessage(seq0[i])
                if i==ind_rand:
                    # expect buffer reset
                    assert(len(ts.queues[0])==0 and len(ts.queues[1])==0)
                m1.signalMessage(seq1[i])
            # expect N synchronisation minus 1 buffer reset
            assert(len(self.collector)==(N-1))


if __name__ == '__main__':
    if 1:
        rostest.unitrun('camera_calibration', 'testapproxsync', TestApproxSync)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestApproxSync('test_approx'))
        unittest.TextTestRunner(verbosity=2).run(suite)
