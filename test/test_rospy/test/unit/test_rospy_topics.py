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

import os
import sys
import struct
import unittest
import time

import test_rospy.msg

def callback1(data): pass
def callback2(data): pass

# for use with publish() tests
import rospy.impl.transport
class ConnectionOverride(rospy.impl.transport.Transport):
    def __init__(self, endpoint_id):
        super(ConnectionOverride, self).__init__(rospy.impl.transport.OUTBOUND, endpoint_id)
        self.endpoint_id = endpoint_id
        self.data = b''

    def set_cleanup_callback(self, cb): pass
    def write_data(self, data):
        self.data = self.data + data

# test rospy API verifies that the rospy module exports the required symbols
class TestRospyTopics(unittest.TestCase):

    def test_add_connection(self):
        from rospy.topics import _TopicImpl
        from std_msgs.msg import String
        t = _TopicImpl('/foo', String)
        c1 = ConnectionOverride('c1')
        c1b = ConnectionOverride('c1')
        c2 = ConnectionOverride('c2')
        c3 = ConnectionOverride('c3')
        assert not t.has_connection('c1')
        assert t.get_num_connections() == 0
        t.add_connection(c1)
        assert t.get_num_connections() == 1
        assert t.has_connection('c1')        
        t.add_connection(c1b)
        assert t.get_num_connections() == 1
        assert t.has_connection('c1')
        # should not remove
        t.remove_connection(c1)
        assert t.has_connection('c1')
        t.remove_connection(c1b)        
        assert not t.has_connection('c1')

        t.close()
        assert t.get_num_connections() == 0 

        t = _TopicImpl('/foo', String)
        t.add_connection(c1)
        t.add_connection(c2)
        t.add_connection(c3)
        for x in ['c1', 'c2', 'c3']:
            assert t.has_connection(x)
        assert t.get_num_connections() == 3

        t.close()
        assert t.get_num_connections() == 0        
        
    def test_Publisher(self):
        import rospy
        from rospy.impl.registration import get_topic_manager, Registration
        from rospy.topics import Publisher, DEFAULT_BUFF_SIZE
        # Publisher(self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None)

        name = 'foo'
        rname = rospy.resolve_name('foo')
        data_class = test_rospy.msg.Val

        # test invalid params
        for n in [None, '', 1]:
            try:
                Publisher(n, data_class)
                self.fail("should not allow invalid name")
            except ValueError: pass
        for d in [None, 1, TestRospyTopics]:
            try:
                Publisher(name, d)
                self.fail("should now allow invalid data_class")
            except ValueError: pass
        try:
            Publisher(name, None)
            self.fail("None should not be allowed for data_class")
        except ValueError: pass

        # round 1: test basic params
        pub = Publisher(name, data_class)
        self.assertEqual(rname, pub.resolved_name)
        # - pub.name is left in for backwards compatibility, but resolved_name is preferred
        self.assertEqual(rname, pub.name)        
        self.assertEqual(data_class, pub.data_class)
        self.assertEqual('test_rospy/Val', pub.type)
        self.assertEqual(data_class._md5sum, pub.md5sum)
        self.assertEqual(Registration.PUB, pub.reg_type)
        
        # verify impl as well
        impl = get_topic_manager().get_impl(Registration.PUB, rname)
        self.assertTrue(impl == pub.impl)
        self.assertEqual(rname, impl.resolved_name)
        self.assertEqual(data_class, impl.data_class)                
        self.assertFalse(impl.is_latch)
        self.assertEqual(None, impl.latch)                
        self.assertEqual(0, impl.seq)
        self.assertEqual(1, impl.ref_count)
        self.assertEqual(b'', impl.buff.getvalue())
        self.assertFalse(impl.closed)
        self.assertFalse(impl.has_connections())
        # check publish() fall-through
        from test_rospy.msg import Val
        impl.publish(Val('hello world-1'))
        
        # check stats
        self.assertEqual(0, impl.message_data_sent)
        # check acquire/release don't bomb
        impl.acquire()
        impl.release()        

        # do a single publish with connection override. The connection
        # override is a major cheat as the object isn't even an actual
        # connection. I will need to add more integrated tests later
        co1 = ConnectionOverride('co1')
        self.assertFalse(impl.has_connection('co1'))
        impl.add_connection(co1)
        self.assertTrue(impl.has_connection('co1'))
        self.assertTrue(impl.has_connections())        
        impl.publish(Val('hello world-1'), connection_override=co1)

        try:
            from cStringIO import StringIO
        except ImportError:
            from io import BytesIO as StringIO
        buff = StringIO()
        Val('hello world-1').serialize(buff)
        # - check equals, but strip length field first
        self.assertEqual(co1.data[4:], buff.getvalue())
        self.assertEqual(None, impl.latch)
        
        # Now enable latch
        pub = Publisher(name, data_class, latch=True)
        impl = get_topic_manager().get_impl(Registration.PUB, rname)
        # have to verify latching in pub impl
        self.assertTrue(impl == pub.impl)
        self.assertEqual(True, impl.is_latch)
        self.assertEqual(None, impl.latch)
        self.assertEqual(2, impl.ref_count)        

        co2 = ConnectionOverride('co2')
        self.assertFalse(impl.has_connection('co2'))
        impl.add_connection(co2)
        for n in ['co1', 'co2']:
            self.assertTrue(impl.has_connection(n))
        self.assertTrue(impl.has_connections())        
        v = Val('hello world-2')
        impl.publish(v, connection_override=co2)
        self.assertTrue(v == impl.latch)

        buff = StringIO()
        Val('hello world-2').serialize(buff)
        # - strip length and check value
        self.assertEqual(co2.data[4:], buff.getvalue())

        # test that latched value is sent to connections on connect
        co3 = ConnectionOverride('co3')
        self.assertFalse(impl.has_connection('co3'))
        impl.add_connection(co3)
        for n in ['co1', 'co2', 'co3']:
            self.assertTrue(impl.has_connection(n))
        self.assertTrue(impl.has_connections())
        self.assertEqual(co3.data[4:], buff.getvalue())        
        
        # TODO: tcp_nodelay
        # TODO: suscribe listener
        self.assertTrue(impl.has_connection('co1'))
        impl.remove_connection(co1)
        self.assertFalse(impl.has_connection('co1'))
        self.assertTrue(impl.has_connections())
        
        self.assertTrue(impl.has_connection('co3'))
        impl.remove_connection(co3)        
        self.assertFalse(impl.has_connection('co3'))
        self.assertTrue(impl.has_connections())
        
        self.assertTrue(impl.has_connection('co2'))
        impl.remove_connection(co2)        
        self.assertFalse(impl.has_connection('co2'))
        self.assertFalse(impl.has_connections())

        # test publish() latch on a new Publisher object (this was encountered in testing, so I want a test case for it)
        pub = Publisher('bar', data_class, latch=True, queue_size=0)
        v = Val('no connection test')
        pub.impl.publish(v)
        self.assertTrue(v == pub.impl.latch)

        # test connection header
        h = {'foo': 'bar', 'fuga': 'hoge'}
        pub = Publisher('header_test', data_class, headers=h, queue_size=0)
        self.assertEqual(h, pub.impl.headers)
        
    def test_Subscriber_unregister(self):
        # regression test for #3029 (unregistering a Subcriber with no
        # callback) plus other unregistration tests
        import rospy
        from rospy.impl.registration import get_topic_manager, Registration
        from rospy.topics import Subscriber, DEFAULT_BUFF_SIZE

        #Subscriber: name, data_class, callback=None, callback_args=None,
        #queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):

        name = 'unregistertest'
        rname = rospy.resolve_name(name)
        data_class = test_rospy.msg.Val

        sub = Subscriber(name, data_class)
        self.assertEqual(None, sub.callback)

        # verify impl (test_Subscriber handles more verification, we
        # just care about callbacks and ref_count state here)
        impl = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assertTrue(impl == sub.impl)
        self.assertEqual(1, impl.ref_count)
        self.assertEqual([], impl.callbacks)
        
        # unregister should release the underlying impl
        sub.unregister()
        self.assertEqual(None, get_topic_manager().get_impl(Registration.SUB, rname))

        # create two subs
        sub2 = Subscriber(name, data_class)
        sub3 = Subscriber(name, data_class)        

        impl = get_topic_manager().get_impl(Registration.SUB, rname)
        # - test that they share the same impl
        self.assertTrue(impl == sub2.impl)
        self.assertTrue(impl == sub3.impl)
        # - test basic impl state
        self.assertEqual([], impl.callbacks)
        self.assertEqual(2, impl.ref_count)
        sub2.unregister()
        self.assertEqual(1, impl.ref_count)
        # - make sure double unregister is safe
        sub2.unregister()
        self.assertEqual(1, impl.ref_count)
        # - clean it up
        sub3.unregister()
        self.assertEqual(0, impl.ref_count)
        self.assertEqual(None, get_topic_manager().get_impl(Registration.SUB, rname))

        # CALLBACKS
        cb_args5 = 5
        cb_args6 = 6
        cb_args7 = 7
        sub4 = Subscriber(name, data_class, callback1)
        # - use should be allowed to subcribe using the same callback
        # and it shouldn't interfere on unregister
        sub5 = Subscriber(name, data_class, callback2, cb_args5)        
        sub6 = Subscriber(name, data_class, callback2, cb_args6)        
        sub7 = Subscriber(name, data_class, callback2, cb_args7)        
        impl = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assertEqual(4, impl.ref_count)
        
        self.assertEqual([(callback1, None), (callback2, cb_args5), (callback2, cb_args6), (callback2, cb_args7)], impl.callbacks)
        # unregister sub6 first to as it is most likely to confuse any callback-finding logic
        sub6.unregister()
        self.assertEqual([(callback1, None), (callback2, cb_args5), (callback2, cb_args7)], impl.callbacks)
        self.assertEqual(3, impl.ref_count)
        sub5.unregister()
        self.assertEqual([(callback1, None), (callback2, cb_args7)], impl.callbacks)
        self.assertEqual(2, impl.ref_count)
        sub4.unregister()
        self.assertEqual([(callback2, cb_args7)], impl.callbacks)
        self.assertEqual(1, impl.ref_count)
        sub7.unregister()
        self.assertEqual([], impl.callbacks)
        self.assertEqual(0, impl.ref_count)
        self.assertEqual(None, get_topic_manager().get_impl(Registration.SUB, rname))

        # one final condition: two identical subscribers
        sub8 = Subscriber(name, data_class, callback1, 'hello')
        sub9 = Subscriber(name, data_class, callback1, 'hello')
        impl = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assertEqual([(callback1, 'hello'), (callback1, 'hello')], impl.callbacks)
        self.assertEqual(2, impl.ref_count)
        sub8.unregister()
        self.assertEqual([(callback1, 'hello')], impl.callbacks)
        self.assertEqual(1, impl.ref_count)
        sub9.unregister()
        self.assertEqual([], impl.callbacks)
        self.assertEqual(0, impl.ref_count)

    def test_Subscriber(self):
        #TODO: test callback args
        #TODO: negative buff_size
        #TODO: negative queue_size        
        import rospy
        from rospy.impl.registration import get_topic_manager, Registration
        from rospy.topics import Subscriber, DEFAULT_BUFF_SIZE

        #Subscriber: name, data_class, callback=None, callback_args=None,
        #queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):

        name = 'foo'
        rname = rospy.resolve_name('foo')
        data_class = test_rospy.msg.Val

        # test invalid params
        for n in [None, '', 1]:
            try:
                Subscriber(n, data_class)
                self.fail("should not allow invalid name")
            except ValueError: pass
        for d in [None, 1, TestRospyTopics]:
            try:
                Subscriber(name, d)
                self.fail("should now allow invalid data_class")
            except ValueError: pass
        try:
            Subscriber(name, None)
            self.fail("None should not be allowed for data_class")
        except ValueError: pass
        
        sub = Subscriber(name, data_class)
        self.assertEqual(rname, sub.resolved_name)
        self.assertEqual(data_class, sub.data_class)
        self.assertEqual('test_rospy/Val', sub.type)
        self.assertEqual(data_class._md5sum, sub.md5sum)
        self.assertEqual(Registration.SUB, sub.reg_type)
        
        # verify impl as well
        impl = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assertTrue(impl == sub.impl)
        self.assertEqual([], impl.callbacks)
        self.assertEqual(rname, impl.resolved_name)
        self.assertEqual(data_class, impl.data_class)                
        self.assertEqual(None, impl.queue_size)
        self.assertEqual(DEFAULT_BUFF_SIZE, impl.buff_size)
        self.assertFalse(impl.tcp_nodelay)
        self.assertEqual(1, impl.ref_count)
        self.assertFalse(impl.closed)
        
        # round 2, now start setting options and make sure underlying impl is reconfigured
        name = 'foo'
        data_class = test_rospy.msg.Val
        queue_size = 1
        buff_size = 1
        sub = Subscriber(name, data_class, callback=callback1,
                         queue_size=queue_size, buff_size=buff_size, tcp_nodelay=True)
        self.assertEqual(rname, sub.resolved_name)
        # - sub.name is a backwards-compat field as it is public API
        self.assertEqual(rname, sub.name)        
        self.assertEqual(data_class, sub.data_class)

        # verify impl 
        impl2 = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assertTrue(impl == impl2) # should be same instance
        self.assertEqual([(callback1, None)], impl.callbacks)
        self.assertEqual(rname, impl.resolved_name)
        self.assertEqual(data_class, impl.data_class)                
        self.assertEqual(queue_size, impl.queue_size)
        self.assertEqual(buff_size, impl.buff_size)
        self.assertTrue(impl.tcp_nodelay)
        self.assertEqual(2, impl.ref_count)
        self.assertFalse(impl.closed)

        # round 3, make sure that options continue to reconfigure
        # underlying impl also test that tcp_nodelay is sticky. this
        # is technically undefined, but this is how rospy chose to
        # implement.
        name = 'foo'
        data_class = test_rospy.msg.Val
        queue_size = 2
        buff_size = 2
        sub = Subscriber(name, data_class, callback=callback2, 
                         queue_size=queue_size, buff_size=buff_size, tcp_nodelay=False)

        # verify impl 
        impl2 = get_topic_manager().get_impl(Registration.SUB, rname)
        self.assertTrue(impl == impl2) # should be same instance
        self.assertEqual(set([(callback1, None), (callback2, None)]), set(impl.callbacks))
        self.assertEqual(queue_size, impl.queue_size)
        self.assertEqual(buff_size, impl.buff_size)
        self.assertTrue(impl.tcp_nodelay)
        self.assertEqual(3, impl.ref_count)
        self.assertFalse(impl.closed)

    def test_Poller(self):
        # no real test as this goes down to kqueue/select, just make sure that it behaves
        from rospy.topics import Poller
        p = Poller()
        p.add_fd(1)
        for x in p.error_iter():
            pass
        p.remove_fd(1)
        for x in p.error_iter():
            pass
        
