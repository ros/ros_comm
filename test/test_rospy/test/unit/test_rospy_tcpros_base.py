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
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
        
import rospy

g_fileno = 1

class MockSock:
    def __init__(self, buff):
        global g_fileno
        g_fileno += 1
        self.buff = buff
        self._fileno = g_fileno
    def fileno(self):
        return self._fileno
    def recv(self, buff_size):
        return self.buff[:buff_size]
    def close(self):
        self.buff = None
    def getsockname(self):
        return (None, None)
class MockEmptySock:
    def __init__(self):
        global g_fileno
        g_fileno += 1
        self._fileno = g_fileno
    def fileno(self):
        return self._fileno
    def recv(self, buff_size):
        return ''
    def close(self):
        self.buff = None

class TestRospyTcprosBase(unittest.TestCase):

    def test_constants(self):
        self.assertEqual("TCPROS", rospy.impl.tcpros_base.TCPROS)
        self.assertTrue(type(rospy.impl.tcpros_base.DEFAULT_BUFF_SIZE), int)

    def test_recv_buff(self):
        from rospy.impl.tcpros_base import recv_buff


        buff = StringIO()
        try:
            recv_buff(MockEmptySock(), buff, 1)
            self.fail("recv_buff should have raised TransportTerminated")
        except rospy.impl.tcpros_base.TransportTerminated:
            self.assertEqual('', buff.getvalue())

        self.assertEqual(5, recv_buff(MockSock('1234567890'), buff, 5))
        self.assertEqual('12345', buff.getvalue())
        buff = StringIO()
        
        self.assertEqual(10, recv_buff(MockSock('1234567890'), buff, 100))
        self.assertEqual('1234567890', buff.getvalue())

    def test_TCPServer(self):
        from rospy.impl.tcpros_base import TCPServer
        def handler(sock, addr):
            pass
        s = None
        try:
            s = TCPServer(handler)
            self.assertTrue(s.port > 0)
            addr, port = s.get_full_addr()
            self.assertTrue(type(addr) == str)
            self.assertEqual(handler, s.inbound_handler)        
            self.assertFalse(s.is_shutdown)
        finally:
            if s is not None:
                s.shutdown()
                self.assertTrue(s.is_shutdown)

    def test_TCPROSTransportProtocol(self):
        import rospy
        import random

        from rospy.impl.tcpros_base import TCPROSTransportProtocol
        from rospy.impl.transport import BIDIRECTIONAL
        
        p = TCPROSTransportProtocol('Bob', rospy.AnyMsg)
        self.assertEqual('Bob', p.resolved_name)
        self.assertEqual(rospy.AnyMsg, p.recv_data_class)
        self.assertEqual(BIDIRECTIONAL, p.direction)
        self.assertEqual({}, p.get_header_fields())
        self.assertEqual(rospy.impl.tcpros_base.DEFAULT_BUFF_SIZE, p.buff_size)

        v = random.randint(1, 100)
        p = TCPROSTransportProtocol('Bob', rospy.AnyMsg, queue_size=v)
        self.assertEqual(v, p.queue_size)

        v = random.randint(1, 100)        
        p = TCPROSTransportProtocol('Bob', rospy.AnyMsg, buff_size=v)
        self.assertEqual(v, p.buff_size)

    def test_TCPROSTransport(self):
        import rospy.impl.tcpros_base
        from rospy.impl.tcpros_base import TCPROSTransport, TCPROSTransportProtocol
        from rospy.impl.transport import OUTBOUND
        p = TCPROSTransportProtocol('Bob', rospy.AnyMsg)
        p.direction = OUTBOUND

        try:
            TCPROSTransport(p, '')
            self.fail("TCPROSTransport should not accept bad name")
        except rospy.impl.tcpros_base.TransportInitError: pass
        
        t = TCPROSTransport(p, 'transport-name')
        self.assertTrue(t.socket is None)
        self.assertTrue(t.md5sum is None)
        self.assertTrue(t.type is None)         
        self.assertEqual(p, t.protocol)
        self.assertEqual('TCPROS', t.transport_type)        
        self.assertEqual(OUTBOUND, t.direction)        
        self.assertEqual('unknown', t.endpoint_id)        
        self.assertEqual(b'', t.read_buff.getvalue())
        self.assertEqual(b'', t.write_buff.getvalue())

        s = MockSock('12345')
        t.set_socket(s, 'new_endpoint_id')
        self.assertEqual('new_endpoint_id', t.endpoint_id)
        self.assertEqual(s, t.socket)

        t.close()
        self.assertTrue(t.socket is None)
        self.assertTrue(t.read_buff is None)
        self.assertTrue(t.write_buff is None)
        self.assertTrue(t.protocol is None)
