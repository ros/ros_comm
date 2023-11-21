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

import sys
import unittest
from subprocess import CalledProcessError, check_call

import rostest

PKG = 'roslaunch'
NAME = 'test_roslaunch_exit_code'


class TestRoslaunchExit(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def test_roslaunch(self):
        # network is initialized
        cmd = 'roslaunch'
        exit_code = 37

        check_call([cmd, 'test_roslaunch', 'exit.launch',
                    'exit_code:={}'.format(0), 'required:=true'])

        try:
            check_call([cmd, 'test_roslaunch', 'exit.launch',
                        'exit_code:={}'.format(exit_code), 'required:=true'])
        except CalledProcessError as ex:
            if ex.returncode != exit_code:
                raise(ex)

        check_call([cmd, 'test_roslaunch', 'exit.launch',
                    'exit_code:={}'.format(0), 'required:=false'])
        check_call([cmd, 'test_roslaunch', 'exit.launch',
                    'exit_code:={}'.format(exit_code), 'required:=false'])


if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRoslaunchExit, sys.argv)
