#!/usr/bin/env python
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

## Simple demo of a rospy service that add two integers

NAME = 'add_two_ints_server'

# import the TwoInts service
from test_rosuds.srv import *
import rospy
import logging
import time

sleep_time = 0

class RosLogHandler(logging.Handler):
   def emit(self, record):
      print("[%s:%d:%s] %s"%(record.filename,record.lineno, record.funcName, record.getMessage()))

def set_log(name):
    logger = logging.getLogger(name)
    logger.addHandler(RosLogHandler())
    logger.setLevel(logging.DEBUG)

def add_two_ints(req):
    time.sleep(sleep_time)
    print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    return TwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    # log setting for debug
    # set_log('rospy.tcpros')
    # set_log('rospy.client')
    # set_log('rospy.init')
    # set_log('rospy.registration')
    # set_log('rospy.impl.masterslave')
    # set_log('rospy.service')

    rospy.init_node(NAME)
    s = rospy.Service('add_two_ints_uds', TwoInts, add_two_ints)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    argv = rospy.myargv()
    if len(argv) == 2:
        sleep_time = int(argv[1])
    add_two_ints_server()
