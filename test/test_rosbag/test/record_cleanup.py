#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2020 Amazon.com, Inc. or its affiliates.
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

import roslib
roslib.load_manifest('rosbag')

import os
import unittest
import rostest
import sys
import time
import signal
import subprocess

RECORD_COMMAND = ['rosbag',
                  'record',
                  'chatter',
                  '-O',
                  '--duration=5']
SLEEP_TIME_SEC = 10

class RecordCleanup(unittest.TestCase):

  def test_sigint_cleanup(self):
    """
    Test that rosbag cleans up after handling SIGINT
    """
    test_bag_file_name = '/tmp/record_sigint_cleanup_test.bag'
    test_signal_cleanup(test_bag_file_name, signal.SIGINT)

    # check that the recorded file is no longer active
    self.assertTrue(os.path.isfile(test_bag_file_name))
    self.assertFalse(os.path.isfile(test_bag_file_name+ '.active'))

  def test_sigterm_cleanup(self):
    """
    Test that rosbag cleans up after handling SIGTERM
    """
    test_bag_file_name = '/tmp/record_sigterm_cleanup_test.bag'
    test_signal_cleanup(test_bag_file_name, signal.SIGTERM)

    # check that the recorded file is no longer active
    self.assertTrue(os.path.isfile(test_bag_file_name))
    self.assertFalse(os.path.isfile(test_bag_file_name+ '.active'))


def test_signal_cleanup(test_bag_file_name, signal_to_test):
  """
  Helper method to start rosbag record, send it a signal to stop, and check to see if
  it cleaned up as expected.

  :param test_bag_file_name: the output bag file
  :param signal_to_test: the signal to send to rosbag
  :return:
  """
  test_command = list(RECORD_COMMAND)
  test_command.insert(4, test_bag_file_name)

  p = subprocess.Popen(test_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  # wait while the recorder creates a bag for us to examine
  time.sleep(SLEEP_TIME_SEC)
  p.send_signal(signal_to_test)
  p.wait()


if __name__ == '__main__':
  rostest.unitrun('test_rosbag', 'test_sigint_cleanup', RecordCleanup, sys.argv)
  rostest.unitrun('test_rosbag', 'test_sigterm_cleanup', RecordCleanup, sys.argv)
