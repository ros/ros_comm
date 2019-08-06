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
import time
        
from subprocess import Popen, PIPE, check_call, call

class TestRostopicOffline(unittest.TestCase):

    def setUp(self):
        pass

    ## test that the rosmsg command works
    def test_cmd_help(self):
        cmd = 'rostopic'

        sub = ['bw', 'echo', 'hz', 'delay', 'info', 'list', 'pub', 'type','find']
        output = Popen([cmd], stdout=PIPE).communicate()[0].decode()
        self.assert_('Commands' in output)
        output = Popen([cmd, '-h'], stdout=PIPE).communicate()[0].decode()
        self.assert_('Commands' in output)
        # make sure all the commands are in the usage
        for c in sub:
            cmd_sub = "%s %s"%(cmd, c)
            self.assert_(cmd_sub in output, "'%s' is not in help: \n%s"%(cmd_sub, output))

        for c in sub:
            output = Popen([cmd, c, '-h'], stdout=PIPE, stderr=PIPE).communicate()
            self.assert_("usage:" in output[0].decode().lower(), output)
            # make sure usage refers to the command
            self.assert_("%s %s"%(cmd, c) in output[0].decode(), output)
            
        # test no args on commands that require args
        for c in ['bw', 'echo', 'hz', 'delay', 'info', 'pub', 'type', 'find']:
            output = Popen([cmd, c], stdout=PIPE, stderr=PIPE).communicate()
            self.assert_("usage:" in output[0].decode().lower() or "usage:" in output[1].decode().lower(), output)
            # make sure usage refers to the command
            self.assert_("%s %s"%(cmd, c) in output[1].decode(), output)
            
    def test_offline(self):
        cmd = 'rostopic'

        # point at a different 'master'
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = 'http://localhost:11312'
        kwds = { 'env': env, 'stdout': PIPE, 'stderr': PIPE}

        msg = "ERROR: Unable to communicate with master!\n"

        output = Popen([cmd, 'bw', 'chatter'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'echo', 'chatter'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'hz', 'chatter'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'delay', 'chatter'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'list'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'pub', 'chatter', 'std_msgs/String', 'hello'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'type', 'chatter'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
        output = Popen([cmd, 'type', 'std_msgs/String'], **kwds).communicate()[1].decode()
        self.assert_(output.endswith(msg))
