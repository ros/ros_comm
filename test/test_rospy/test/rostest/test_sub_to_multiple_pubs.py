#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
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

## Integration test for subscribing to a topic with multiple publishers

from __future__ import print_function

PKG = 'test_rospy'
NAME = 'test_sub_to_multiple_pubs'

import socket
import sys
import time
import unittest

import rosgraph
from rosgraph.xmlrpc import ServerProxy
import rospy
import rostest

TOPIC = '/chatter'
TALKER_NODE = 'talker%d'
NUMBER_OF_TALKERS = 99
LISTENER_NODE = 'listener'


class TestPubSubToMultiplePubs(unittest.TestCase):

    def test_subscribe_to_multiple_publishers(self):
        # wait so that all connections are established
        time.sleep(1.0)

        # ensure that publishers are publishing
        for i in range(1, NUMBER_OF_TALKERS + 1):
            self.assert_(rostest.is_publisher(
                rospy.resolve_name(TOPIC),
                rospy.resolve_name(TALKER_NODE % i)), 'talker node %d is not up' % i)

        # ensure that subscriber is subscribed
        self.assert_(rostest.is_subscriber(
            rospy.resolve_name(TOPIC),
            rospy.resolve_name(LISTENER_NODE)), 'listener node is not up')

        # check number of connections from subscriber to the topic
        connections = 0

        master = rosgraph.Master(NAME)
        node_api = master.lookupNode(LISTENER_NODE)
        if not node_api:
            self.assert_(False, 'cannot contact [%s]: unknown node' % LISTENER_NODE)

        socket.setdefaulttimeout(5.0)
        node = ServerProxy(node_api)
        code, _, businfo = node.getBusInfo(NAME)
        if code != 1:
            self.assert_(False, 'cannot get node information')
        if businfo:
            for info in businfo:
                topic = info[4]
                if len(info) > 5:
                    connected = info[5]
                else:
                    connected = True  # backwards compatibility

                if connected:
                    if topic == TOPIC:
                        connections += 1

        self.assert_(connections == NUMBER_OF_TALKERS, 'Found only %d connections instead of %d' % (connections, NUMBER_OF_TALKERS))

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.run(PKG, NAME, TestPubSubToMultiplePubs, sys.argv)
