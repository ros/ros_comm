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

PKG = 'test_rospy'
NAME = 'test_latch_unsubscribe'

import os
import sys
import unittest

import psutil

from std_msgs.msg import String


def _get_connections(process_info):
    if hasattr(process_info, 'connections'):  # new naming
        return process_info.connections()
    elif hasattr(process_info, 'get_connections'):  # old naming
        return process_info.get_connections()
    raise AttributeError('Wrong psutil version?')


def _get_connection_statii(process_info):
    return (conn.status for conn in _get_connections(process_info))


class TestLatch(unittest.TestCase):

    def setUp(self):
        pass

    def test_latch(self):
        import rospy
        proc_info = psutil.Process(os.getpid())
        self.assertNotIn('CLOSE_WAIT', _get_connection_statii(proc_info),
                         'CLOSE_WAIT sockets already before the test. This '
                         'should not happen at all.')

        rospy.init_node(NAME)
        pub = rospy.Publisher('chatter', String, latch=True)
        pub.publish(String("hello"))
        rospy.sleep(0.5)
        self.assertNotIn('CLOSE_WAIT', _get_connection_statii(proc_info),
                         'CLOSE_WAIT sockets after the subscriber exited. '
                         '(#107)')
        rospy.sleep(1.5)
        # also check for a second subscriber
        self.assertNotIn('CLOSE_WAIT', _get_connection_statii(proc_info),
                         'CLOSE_WAIT sockets after the second subscriber '
                         'exited. (#107)')


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestLatch, sys.argv)
