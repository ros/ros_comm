#!/usr/bin/env python
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
import unittest
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import time
        
import rosunit

from subprocess import Popen, PIPE, check_call, call

from contextlib import contextmanager

@contextmanager
def fakestdout():
    realstdout = sys.stdout
    fakestdout = StringIO()
    sys.stdout = fakestdout
    yield fakestdout
    sys.stdout = realstdout
    
def todict(s):
    d = {}
    for l in s.split('\n'):
        key, p, val = l.partition(':')
        if p:
            d[key] = val.strip()
    return d
        
class TestRosservice(unittest.TestCase):

    def setUp(self):
        pass

    def test_get_service_headers(self):
        import rosservice
        orig_uri = os.environ['ROS_MASTER_URI']
        os.environ['ROS_MASTER_URI'] = 'http://fake_host:12356'
        try:
            c = 'rosservice'

            # test error conditions, integration tests cover success cases
            try:
                rosservice.get_service_headers('/add_two_ints', 'fake://localhost:1234')
                self.fail("should have raised")
            except rosservice.ROSServiceException: pass
            try:
                rosservice.get_service_headers('/add_two_ints', 'rosrpc://fake_host:1234')
                self.fail("should have raised IO exc")
            except rosservice.ROSServiceIOException: pass
            
        
        finally:
            os.environ['ROS_MASTER_URI'] = orig_uri
        
    def test_get_service_type(self):
        import rosservice
        self.assertEquals('test_rosmaster/AddTwoInts', rosservice.get_service_type('/add_two_ints'))
        self.assertEquals(None, rosservice.get_service_type('/fake_add_two_ints'))

    def test_offline(self):
        import rosservice
        orig_uri = os.environ['ROS_MASTER_URI']
        os.environ['ROS_MASTER_URI'] = 'http://fake_host:12356'

        try:
            c = 'rosservice'

            try:
                rosservice.get_service_type('/add_two_ints')
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException:
                pass
            
            try:
                rosservice._rosservice_cmd_list([c, 'list'])
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException: pass
            
            try:
                rosservice._rosservice_cmd_info([c, 'info', '/add_two_ints'])
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException: pass

            try:
                rosservice._rosservice_cmd_type([c, 'type', '/add_two_ints'])
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException: pass

            try:
                rosservice._rosservice_cmd_uri([c, 'uri', '/add_two_ints'])
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException: pass

            try:
                rosservice._rosservice_cmd_find([c, 'find', 'test_ros/AddTwoInts'])
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException: pass
            
            try:
                rosservice._rosservice_cmd_call([c, 'call', '/add_two_ints', '1', '2'])
                self.fail("should have raised ROSServiceIOException")
            except rosservice.ROSServiceIOException: pass

        finally:
            os.environ['ROS_MASTER_URI'] = orig_uri
        
    def test_cmd_type(self):
        import rosservice
        cmd = 'rosservice'
        s = '/add_two_ints'
        try:
            rosservice.rosservicemain([cmd, 'type', '/fake_service'])
            self.fail("should have triggered error exit")
        except SystemExit:
            pass

        for s in ['/add_two_ints', 'add_two_ints', 'foo/add_two_ints']:
            with fakestdout() as b:
                rosservice.rosservicemain([cmd, 'type', s])
                v = b.getvalue().strip()
                self.assertEquals('test_rosmaster/AddTwoInts', v)

    def test_cmd_uri(self):
        import rosservice
        cmd = 'rosservice'
        with fakestdout() as b:
            try:
                rosservice.rosservicemain([cmd, 'uri', '/fake_service'])
                self.fail("should have triggered error exit")
            except SystemExit:
                pass

        for s in ['/add_two_ints', 'add_two_ints', 'foo/add_two_ints']:
            with fakestdout() as b:
                rosservice.rosservicemain([cmd, 'uri', s])
                v = b.getvalue().strip()
                self.assert_(v.startswith('rosrpc://'), v)
                

    def test_cmd_node(self):
        import rosservice
        cmd = 'rosservice'
        for s in ['/add_two_ints', 'add_two_ints', 'foo/add_two_ints']:
            with fakestdout() as b:
                rosservice.rosservicemain([cmd, 'node', s])
                v = b.getvalue().strip()
                if 'foo' in s:
                    self.assertEquals('/foo/a2iserver', v)
                else:
                    self.assertEquals('/a2iserver', v)
        try:
            rosservice.rosservicemain([cmd, 'node', '/fake_two_ints'])
            self.fail("should have exited with error")
        except SystemExit: pass

    def test_full_usage(self):
        import rosservice
        try:
            rosservice._fullusage()
            self.fail("should have caused system exit")
        except SystemExit: pass
        
    def test_cmd_info(self):
        import rosservice
        cmd = 'rosservice'

        try:
            rosservice.rosservicemain([cmd, 'info'])
            self.fail("should have exited with error")
        except SystemExit: pass
        try:
            rosservice.rosservicemain([cmd, 'info', '/fake_service'])
            self.fail("should have exited with error")
        except SystemExit: pass
        try:
            rosservice.rosservicemain([cmd, 'info', '/add_two_ints', '/foo/add_two_ints'])
            self.fail("should have exited with error")
        except SystemExit: pass
        
        for s in ['/add_two_ints', 'add_two_ints', 'foo/add_two_ints']:
            with fakestdout() as b:
                rosservice.rosservicemain([cmd, 'info', s])
                d = todict(b.getvalue())
                if 'foo' in s:
                    self.assertEquals('/foo/a2iserver', d['Node'])
                else:
                    self.assertEquals('/a2iserver', d['Node'], repr(d['Node']))
                self.assertEquals('test_rosmaster/AddTwoInts', d['Type'])
                self.assertEquals('a b', d['Args'])
                self.assert_('URI' in d)

    def test_cmd_find(self):
        import rosservice
        cmd = 'rosservice'

        try:
            rosservice.rosservicemain([cmd, 'find'])
            self.fail("arg parsing should have failed")
        except SystemExit: pass
        try:
            rosservice.rosservicemain([cmd, 'find', 'test_ros/AddTwoInts', 'test/AddThreeInts'])
            self.fail("arg parsing should have failed")
        except SystemExit: pass
        
        v = set(['/add_two_ints', '/bar/add_two_ints', '/foo/add_two_ints'])
        with fakestdout() as b:
            rosservice.rosservicemain([cmd, 'find', 'test_rosmaster/AddTwoInts'])
            d = set([x for x in b.getvalue().split('\n') if x.strip()])
            self.assertEquals(v, d)

        with fakestdout() as b:
            rosservice.rosservicemain([cmd, 'find', 'fake/AddTwoInts'])
            self.assertEquals('', b.getvalue().strip())

    def test_get_service_class_by_name(self):
        import rosservice
        try:
            rosservice.get_service_class_by_name('fake')
            self.fail("should have raised")
        except rosservice.ROSServiceException as e:
            self.assertEquals("Service [fake] is not available.", str(e))
        
    def test_cmd_call(self):
        import rosservice
        cmd = 'rosservice'

        try:
            rosservice.rosservicemain([cmd, 'call'])
            self.fail("arg parsing should have failed")
        except SystemExit: pass
        try:
            rosservice.rosservicemain([cmd, 'call', 'add_two_ints', '1', '2', '3'])
            self.fail("should have failed with too many args")
        except SystemExit: pass

            
    def setUp(self):
        # wait for all services to come up

        import rosservice        
        services = ['/add_two_ints',
                    '/foo/add_two_ints',
                    '/bar/add_two_ints',
                    ]

        import time
        timeout_t = time.time() + 10.
        while time.time() < timeout_t:
            with fakestdout() as b:
                rosservice._rosservice_cmd_list(['rosservice', 'list'])
                v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
                if not (set(services) - set(v) ):
                    return
        self.fail("timeout")
        
    def test_cmd_list(self):
        import rosservice
        cmd = 'rosservice'
        s = '/add_two_ints'
        
        # test main entry
        rosservice.rosservicemain([cmd, 'list'])

        # test directly
        services = ['/add_two_ints',
                    '/foo/add_two_ints',
                    '/bar/add_two_ints',
                    '/header_echo',
                    ]
        services_nodes = ['/add_two_ints /a2iserver',
                          '/foo/add_two_ints /foo/a2iserver',
                          '/bar/add_two_ints /bar/a2iserver',
                          '/header_echo /headerserver',
                          ]

        with fakestdout() as b:
            rosservice._rosservice_cmd_list([cmd, 'list'])
            v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
            v = [x for x in v if not x.startswith('/rosout/')]
            v = [x for x in v if not x.endswith('/get_loggers') and not x.endswith('/set_logger_level')]
            self.assertEquals(set(services), set(v))
        with fakestdout() as b:
            rosservice._rosservice_cmd_list([cmd, 'list', '-n'])
            v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
            v = [x for x in v if not x.startswith('/rosout/')]
            v = [x for x in v if x.find('/get_loggers ') == -1 and x.find('/set_logger_level ') == -1]
            self.assertEquals(set(services_nodes), set(v))
        with fakestdout() as b:            
            rosservice._rosservice_cmd_list([cmd, 'list', '--nodes'])
            v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
            v = [x for x in v if not x.startswith('/rosout/')]
            v = [x for x in v if x.find('/get_loggers ') == -1 and x.find('/set_logger_level ') == -1]
            self.assertEquals(set(services_nodes), set(v))

        # test with multiple service names
        try:
            rosservice._rosservice_cmd_list([cmd, 'list', s, s])
            self.fail("should have caused parser error")
        except SystemExit:
            pass

        # test with resolved service names
        for s in services:
            with fakestdout() as b:
                rosservice._rosservice_cmd_list([cmd, 'list', s])
                self.assertEquals(s, b.getvalue().strip())

        # test with relative service names
        s = 'add_two_ints'       
        with fakestdout() as b:
            rosservice._rosservice_cmd_list([cmd, 'list', s])
            self.assertEquals('/add_two_ints', b.getvalue().strip())
        with fakestdout() as b:
            rosservice._rosservice_cmd_list([cmd, 'list', s, '-n'])
            self.assertEquals('/add_two_ints /a2iserver', b.getvalue().strip())
        with fakestdout() as b:            
            rosservice._rosservice_cmd_list([cmd, 'list', s, '--nodes'])
            self.assertEquals('/add_two_ints /a2iserver', b.getvalue().strip())
            
        # test with namespaces
        s = '/foo'
        rosservice._rosservice_cmd_list([cmd, 'list', s])
        rosservice._rosservice_cmd_list([cmd, 'list', s, '-n'])
        rosservice._rosservice_cmd_list([cmd, 'list', s, '--nodes'])
        s = 'foo'
        rosservice._rosservice_cmd_list([cmd, 'list', s])
        rosservice._rosservice_cmd_list([cmd, 'list', s, '-n'])
        rosservice._rosservice_cmd_list([cmd, 'list', s, '--nodes'])
        

NAME = 'test_rosservice'
if __name__ == '__main__':
    rosunit.unitrun('test_rosservice', NAME, TestRosservice, sys.argv, coverage_packages=['rosservice'])
