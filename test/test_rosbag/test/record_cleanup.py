#!/usr/bin/env python
#
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.
#

import roslib
roslib.load_manifest('rosbag')

import os
import unittest
import rostest
import sys
import time
import subprocess

class RecordCleanup(unittest.TestCase):

  def test_sigint_cleanup(self):
    # Wait while the recorder creates a bag for us to examine
    time.sleep(10.0)
    # Send a SIGTERM to the process
    cmd = ['kill', '-2' ,'$(pgrep rosbag)']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()

    # # Check that the recorded file is no longer active
    self.assertTrue(os.path.isfile('/tmp/record_cleanup_test.bag'))
    self.assertFalse(os.path.isfile('/tmp/record_cleanup_test.bag.active'))

  def test_sigterm_cleanup(self):
    # Wait while the recorder creates a bag for us to examine
    time.sleep(10.0)
    # Send a SIGTERM to the process
    cmd = ['kill', '$(pgrep rosbag)']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.communicate()

    # Check that the recorded file is no longer active
    self.assertTrue(os.path.isfile('/tmp/record_cleanup_test.bag'))
    self.assertFalse(os.path.isfile('/tmp/record_cleanup_test.bag.active'))


if __name__ == '__main__':
  rostest.unitrun('test_rosbag', 'test_sigterm_cleanup', RecordCleanup, sys.argv)
  rostest.unitrun('test_rosbag', 'test_sigint_cleanup', RecordCleanup, sys.argv)