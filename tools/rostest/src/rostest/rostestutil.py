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
# Revision $Id$

from __future__ import print_function

"""
rostest helper routines.
"""

# IMPORTANT: no routine here can in anyway cause rospy to be loaded (that includes roslaunch)

import os
import sys
import logging

def printlog(msg, *args):
    if args:
        msg = msg%args
    logging.getLogger('rostest').info(msg)
    print("[ROSTEST]" + msg)
def printlogerr(msg, *args):
    if args:
        msg = msg%args
    logging.getLogger('rostest').error(msg)
    print("[ROSTEST]" + msg, file=sys.stderr)

_errors = None
def getErrors():
    return _errors

# Most of this code has been moved down into rosunit

import rosunit

rostest_name_from_path = rosunit.rostest_name_from_path

def printRostestSummary(result, rostest_results):
    """
    Print summary of rostest results to stdout.
    """
    # TODO: probably can removed this
    global _errors
    _errors = result.errors
    return rosunit.print_runner_summary(result, rostest_results, runner_name='ROSTEST')

printSummary = rosunit.print_unittest_summary
createXMLRunner = rosunit.create_xml_runner
xmlResultsFile = rosunit.xml_results_file    
test_failure_junit_xml = rosunit.junitxml.test_failure_junit_xml
test_success_junit_xml = rosunit.junitxml.test_success_junit_xml
