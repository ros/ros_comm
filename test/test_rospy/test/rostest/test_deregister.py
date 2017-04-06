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

## Integration test for empty services to test serializers
## and transport

from __future__ import print_function

PKG = 'test_rospy'

import sys
import time
import unittest
import gc
import weakref

import rospy
import rostest
from rosgraph.xmlrpc import ServerProxy
from std_msgs.msg import String
from test_rospy.srv import EmptySrv

PUBTOPIC = 'test_unpublish_chatter'
SUBTOPIC = 'test_unsubscribe_chatter'
SERVICE = 'test_unregister_service'

TIMEOUT = 10.0 #seconds

_last_callback = None
def callback(data):
    global _last_callback
    print("message received", data.data)
    _last_callback = data


class TestDeregister(unittest.TestCase):
        
    def test_unpublish(self):
        node_proxy = ServerProxy(rospy.get_node_uri())
        
        _, _, pubs = node_proxy.getPublications('/foo')
        pubs = [p for p in pubs if p[0] != '/rosout']
        self.assert_(not pubs, pubs)
        
        print("Publishing ", PUBTOPIC)
        pub = rospy.Publisher(PUBTOPIC, String, queue_size=1)
        impl = weakref.ref(pub.impl)
        topic = rospy.resolve_name(PUBTOPIC)
        _, _, pubs = node_proxy.getPublications('/foo')
        pubs = [p for p in pubs if p[0] != '/rosout']
        self.assertEquals([[topic, String._type]], pubs, "Pubs were %s"%pubs)

        # publish about 10 messages for fun
        for i in range(0, 10):
            pub.publish(String("hi [%s]"%i))
            time.sleep(0.1)
        
        # begin actual test by unsubscribing
        pub.unregister()
        
        # make sure no new messages are received in the next 2 seconds
        timeout_t = time.time() + 2.0
        while timeout_t < time.time():
            time.sleep(1.0)
        self.assert_(_last_callback is None)

        # verify that close cleaned up master and node state
        _, _, pubs = node_proxy.getPublications('/foo')
        pubs = [p for p in pubs if p[0] != '/rosout']
        self.assert_(not pubs, "Node still has pubs: %s"%pubs)
        n = rospy.get_caller_id()
        self.assert_(not rostest.is_publisher(topic, n), "publication is still active on master")

        # verify that the impl was cleaned up
        gc.collect()
        self.assertIsNone(impl())
        
    def test_unsubscribe(self):
        global _last_callback

        uri = rospy.get_node_uri()
        node_proxy = ServerProxy(uri)
        _, _, subscriptions = node_proxy.getSubscriptions('/foo')
        self.assert_(not subscriptions, 'subscriptions present: %s'%str(subscriptions))
        
        print("Subscribing to ", SUBTOPIC)
        sub = rospy.Subscriber(SUBTOPIC, String, callback)
        topic = rospy.resolve_name(SUBTOPIC)
        _, _, subscriptions = node_proxy.getSubscriptions('/foo')
        self.assertEquals([[topic, String._type]], subscriptions, "Subscriptions were %s"%subscriptions)
        
        # wait for the first message to be received
        timeout_t = time.time() + TIMEOUT
        while _last_callback is None and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(_last_callback is not None, "No messages received from talker")
        
        # begin actual test by unsubscribing
        sub.unregister()

        # clear last callback data, i.e. last message received
        _last_callback = None
        timeout_t = time.time() + 2.0
        
        # make sure no new messages are received in the next 2 seconds
        while timeout_t < time.time():
            time.sleep(1.0)
        self.assert_(_last_callback is None)

        # verify that close cleaned up master and node state
        _, _, subscriptions = node_proxy.getSubscriptions('/foo')

        self.assert_(not subscriptions, "Node still has subscriptions: %s"%subscriptions)
        n = rospy.get_caller_id()
        self.assert_(not rostest.is_subscriber(topic, n), "subscription is still active on master")

    def test_unservice(self):
        import rosgraph
        master = rosgraph.Master('/test_dereg')

        state = master.getSystemState()
        _, _, srv = state
        # filter out rosout services
        #[['/rosout/set_logger_level', ['/rosout']], ['/rosout/get_loggers', ['/rosout']]]
        srv = [s for s in srv if not s[0].startswith('/rosout/') and not s[0].endswith('/get_loggers') and not s[0].endswith('/set_logger_level')]
        self.failIf(srv, srv)

        print("Creating service ", SERVICE)
        service = rospy.Service(SERVICE, EmptySrv, callback)
        # we currently cannot interrogate a node's services, have to rely on master

        # verify that master has service
        state = master.getSystemState()
        _, _, srv = state
        srv = [s for s in srv if not s[0].startswith('/rosout/') and not s[0].endswith('/get_loggers') and not s[0].endswith('/set_logger_level')]
        self.assertEquals(srv, [[rospy.resolve_name(SERVICE), [rospy.get_caller_id()]]])
        
        # begin actual test by unsubscribing
        service.shutdown()

        time.sleep(1.0) # give API 1 second to sync with master
        
        state = master.getSystemState()
        _, _, srv = state
        srv = [s for s in srv if not s[0].startswith('/rosout/') and not s[0].endswith('/get_loggers') and not s[0].endswith('/set_logger_level')]
        self.failIf(srv, srv)

        
if __name__ == '__main__':
    rospy.init_node('test_dereg', disable_rostime=True)
    rostest.run(PKG, 'rospy_deregister', TestDeregister, sys.argv)
