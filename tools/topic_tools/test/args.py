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
#
# Author: Brian Gerkey

# Test that arg-parsing works

PKG = 'topic_tools'

import unittest
import os
import sys
from subprocess import call

rosrun_script = 'rosrun.bat' if sys.platform == 'win32' else 'rosrun'

class TopicToolsTestCase(unittest.TestCase):

    def test_drop_invalid(self):
        cmd = [rosrun_script, 'topic_tools', 'drop']
        self.assertNotEquals(0, call(cmd))
        self.assertNotEquals(0, call(cmd + ['//', '1', '2', 'output']))
        self.assertNotEquals(0, call(cmd + ['input', '1', '2', 'output', 'extra']))
        self.assertNotEquals(0, call(cmd + ['input', '-1', '2', 'output']))
        self.assertNotEquals(0, call(cmd + ['input', '1', '0', 'output']))

    def test_mux_invalid(self):
        cmd = [rosrun_script, 'topic_tools', 'mux']
        self.assertNotEquals(0, call(cmd))
        self.assertNotEquals(0, call(cmd + ['//', 'input']))

    def test_switch_mux_invalid(self):
        cmd = [rosrun_script, 'topic_tools', 'switch_mux']
        self.assertNotEquals(0, call(cmd))
        self.assertNotEquals(0, call(cmd + ['//', 'input']))

    def test_relay_invalid(self):
        cmd = [rosrun_script, 'topic_tools', 'relay']
        self.assertNotEquals(0, call(cmd))
        self.assertNotEquals(0, call(cmd + ['//', 'input']))

if __name__ == "__main__":
    import rostest
    rostest.unitrun(PKG, 'topic_tools_arg_parsing', TopicToolsTestCase)
