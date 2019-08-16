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

PKG = 'roswtf'
NAME = 'test_roswtf_command_line_online'

import os
import signal
import sys 
import time
import unittest

import rospkg
import rospy
import rostest

import std_msgs.msg

from subprocess import Popen, PIPE, check_call, call

def run_for(cmd, secs):
    popen = Popen(cmd, stdout=PIPE, stderr=PIPE, close_fds=True)
    timeout_t = time.time() + secs
    while time.time() < timeout_t:
        time.sleep(0.1)
    os.kill(popen.pid, signal.SIGKILL)
    
class TestRostopicOnline(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    ## test that the rosmsg command works
    def test_cmd_help(self):
        cmd = 'roswtf'
        output = Popen([cmd, '-h'], stdout=PIPE).communicate()[0].decode()
        self.assert_('Options' in output)
            
    def test_offline(self):
        # this test is disabled for now; now that test_roswtf is part
        # of ros_comm, the tricks before that were used no longer work
        cmd = 'roswtf'

        # pass in special test key to roswtf for ROS_PACKAGE_PATH
        env = os.environ.copy()

        rospack = rospkg.RosPack()
        # add all dependencies to ros package path
        pkgs = ['roswtf',
            'rosgraph', 'roslaunch', 'roslib', 'rosnode', 'rosservice',
            'rosbag', 'rosbag_storage', 'roslz4', 'rosconsole', 'roscpp', 'rosgraph_msgs', 'roslang', 'rosmaster', 'rosmsg', 'rosout', 'rosparam', 'rospy', 'rostest', 'rostopic', 'topic_tools', 'xmlrpcpp',
            'std_srvs',  # ros_comm_msgs
            'cpp_common', 'roscpp_serialization', 'roscpp_traits', 'rostime',  # roscpp_core
            'rosbuild', 'rosclean', 'rosunit',  # ros
            'rospack', 'std_msgs', 'message_runtime', 'message_generation', 'gencpp', 'genlisp', 'genpy', 'genmsg', 'catkin',
        ]
        try:
            paths = [rospack.get_path(pkg) for pkg in pkgs]
        except rospkg.ResourceNotFound as e:
            assert False, 'rospkg.ResourceNotFound: ' + str(e)
        try:
            path = rospack.get_path('cmake_modules')
        except rospkg.ResourceNotFound:
            pass
        else:
            paths.append(path)
        try:
            path = rospack.get_path('geneus')
        except rospkg.ResourceNotFound:
            pass
        else:
            paths.append(path)
        try:
            path = rospack.get_path('gennodejs')
        except rospkg.ResourceNotFound:
            pass
        else:
            paths.append(path)
        env['ROS_PACKAGE_PATH'] = os.pathsep.join(paths)

        cwd  = rospack.get_path('roswtf')
        kwds = { 'env': env, 'stdout': PIPE, 'stderr': PIPE, 'cwd': cwd}

        # run roswtf nakedly in the roswtf directory. Running in
        # ROS_ROOT effectively make roswtf have dependencies on
        # every package in the ROS stack, which doesn't work.

        output, err = Popen([cmd], **kwds).communicate()
        self._check_output([cmd], output.decode(), err.decode())

        # run roswtf on a simple launch file online
        rospack = rospkg.RosPack()
        p = os.path.join(rospack.get_path('roswtf'), 'test', 'min.launch')
        output = Popen([cmd, p], **kwds).communicate()[0].decode()
        self._check_output([cmd, p], output)

    def _check_output(self, cmd, output, error=None):
        # do both a positive and negative test
        self.assert_(
            'No errors or warnings' in output or 'Found 1 error' in output,
            'CMD[%s] OUTPUT[%s]%s' %
            (' '.join(cmd), output, '\nstderr[%s]' % error if error else ''))
        allowed_errors = 0
        if 'Found 1 error' in output:
            self.assert_(output.count('ERROR') == 1, 'OUTPUT[%s]' % output)
            self.assert_(
                'ROS Dep database not updated' in output,
                'OUTPUT[%s]' % output)
            allowed_errors += 1
        if 'No errors or warnings' in output:
            self.assert_(output.count('ERROR') <= allowed_errors, 'OUTPUT[%s]' % output)

if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRostopicOnline, sys.argv)
