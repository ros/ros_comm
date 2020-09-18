#!/usr/bin/env python
# Software License Agreement (Apache 2.0 License)
#
# Copyright (c) 2020, Tecnalia.
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
#  * Neither the name of Tecnalia nor the names of its
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

"""
@package node_test
@file dummy filter
@author Anthony Remazeilles <anthony.remazeilles@tecnalia.com>
@brief dummy prb to check filter testing

Copyright (C) 2020 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.

"""


import rospy
from std_msgs.msg import Float32


class DummyFilterNode():
    def __init__(self):
        self.pub = None
        self.sub = None
        self.wait = None

        rospy.init_node('dummy_filter', anonymous=True)

        self.wait = rospy.get_param('~wait', self.wait)
        rospy.loginfo("waiting time: {}".format(self.wait))

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
        data.data = data.data * 2.0
        if self.wait is not None:
            rospy.loginfo("Sleeping!")
            duration = rospy.Duration(self.wait)
            rospy.sleep(duration)
        self.pub.publish(data)

    def run(self):
        self.pub = rospy.Publisher('filter_out', Float32, queue_size=10)
        self.sub = rospy.Subscriber('filter_in', Float32, self.callback)

        rospy.spin()


if __name__ == '__main__':
    try:
        dummy = DummyFilterNode()
        dummy.run()
    except rospy.ROSInterruptException:
        pass
