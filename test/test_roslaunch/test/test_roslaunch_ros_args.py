#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, The Johns Hopkins University
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

PKG = 'roslaunch'
NAME = 'test_roslaunch_ros_args'

import os
import sys 
import unittest

import rostest

import rospkg
import roslaunch.arg_dump

class TestRoslaunchRosArgs(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def test_roslaunch(self):
        rospack = rospkg.RosPack()
        filename = os.path.join(rospack.get_path('test_roslaunch'), 'test', 'ros_args.launch')

        args = roslaunch.arg_dump.get_args([filename])

        expected_args = {
                'foo': ("I pity the foo'.", 'true'),
                'bar': ("Someone walks into this.", None,),
                'baz': (None, 'false'),
                'nop': (None, None)}

        self.assertEqual(args, expected_args)

if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRoslaunchRosArgs, sys.argv)
