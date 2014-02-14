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
import time
import unittest

import rospy
import rostest

import test_rosmaster.srv

from subprocess import Popen, PIPE, check_call, call

from threading import Thread
class TestTask(Thread):
    def __init__(self, task):
        Thread.__init__(self)
        self.task = task
        self.success = False
        self.done = False
        self.value = None
        
    def run(self):
        try:
            print("STARTING TASK")
            self.value = self.task()
            print("TASK HAS COMPLETED")
            self.success = True
        except:
            import traceback
            traceback.print_exc()
        self.done = True

class TestRosserviceOnline(unittest.TestCase):

    def setUp(self):
        pass
        
    def test_rosservice(self):
        # wait for network to initialize
        services = ['/add_two_ints', '/foo/add_two_ints', '/bar/add_two_ints']
        for s in services:
            rospy.wait_for_service(s)

        cmd = 'rosservice'
        names = ['add_two_ints', '/add_two_ints', 'foo/add_two_ints', '/bar/add_two_ints']

        # list
        # - hard to exact match as we are still adding builtin services to nodes (e.g. set_logger_level)
        output = Popen([cmd, 'list'], stdout=PIPE).communicate()[0]
        output = output.decode()
        l = set(output.split())
        for s in services:
            self.assert_(s in l)

        for name in names:
            # args
            output = Popen([cmd, 'args', name], stdout=PIPE).communicate()[0]
            self.assertEquals('a b', output.strip())

            # type
            output = Popen([cmd, 'type', name], stdout=PIPE).communicate()[0]
            self.assertEquals('test_rosmaster/AddTwoInts', output.strip())

            # find
            output = Popen([cmd, 'find', 'test_rosmaster/AddTwoInts'], stdout=PIPE).communicate()[0]
            values = [v.strip() for v in output.split('\n') if v.strip()]
            self.assertEquals(set(values), set(services))

            # uri
            output = Popen([cmd, 'uri', name], stdout=PIPE).communicate()[0]
            # - no exact answer
            self.assert_(output.startswith('rosrpc://'), output)

            # call
            output = Popen([cmd, 'call', '--wait', name, '1', '2'], stdout=PIPE).communicate()[0]
            self.assertEquals('sum: 3', output.strip())

            output = Popen([cmd, 'call', name, '1', '2'], stdout=PIPE).communicate()[0]
            self.assertEquals('sum: 3', output.strip())

        name = 'header_echo'
        # test with a Header so we can validate keyword args
        import yaml
        import time
        t = time.time()
        
        # test with empty headers
        for v in ['{}', '{header: {}}', '{header: {seq: 0}}']:
            output = Popen([cmd, 'call', name, v], stdout=PIPE).communicate()[0]
            output = output.strip()
            self.assert_(output, output)
            val = yaml.load(output)['header']
            self.assertEquals('', val['frame_id'])
            self.assert_(val['seq'] >= 0)
            self.assertEquals(0, val['stamp']['secs'])
            self.assertEquals(0, val['stamp']['nsecs'])

        # test with auto headers
        for v in ['{header: auto}', '{header: {stamp: now}}']:
            output = Popen([cmd, 'call', name, v], stdout=PIPE).communicate()[0]
            val = yaml.load(output.strip())['header']
            self.assertEquals('', val['frame_id'])
            self.assert_(val['seq'] >= 0)
            self.assert_(val['stamp']['secs'] >= int(t))
        

        # verify that it respects ROS_NS
        # - the uris should be different as the names should resolve differently
        env = os.environ.copy()
        env['ROS_NAMESPACE'] = 'foo'
        uri1 = Popen([cmd, 'uri', 'add_two_ints'], stdout=PIPE).communicate()[0]
        uri2 = Popen([cmd, 'uri', 'add_two_ints'], env=env, stdout=PIPE).communicate()[0]
        self.assert_(uri2.startswith('rosrpc://'))
        self.assertNotEquals(uri1, uri2)

        # test_call_wait
        def task1():
            output = Popen([cmd, 'call', '--wait', 'wait_two_ints', '1', '2'], stdout=PIPE).communicate()[0]
            self.assertEquals('sum: 3', output.strip())
        timeout_t = time.time() + 5.
        t1 = TestTask(task1)
        t1.start()
        
        rospy.init_node('test_call_wait')
        rospy.Service("wait_two_ints", test_rosmaster.srv.AddTwoInts, lambda x: x.a + x.b)
        while not t1.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t1.success)
        

PKG = 'test_rosservice'
NAME = 'test_rosservice_command_line_online'
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRosserviceOnline, sys.argv)
