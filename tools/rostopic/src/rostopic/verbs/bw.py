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

from __future__ import division, print_function

import rospy
import sys
import traceback


class ROSTopicBandwidth(object):
    def __init__(self, window_size=100):
        import threading
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.sizes =[]
        self.times =[]
        self.window_size = window_size or 100

    def callback(self, data):
        """ros sub callback"""
        with self.lock:
            try:
                t = time.time()
                self.times.append(t)
                self.sizes.append(len(data._buff)) #AnyMsg instance
                assert(len(self.times) == len(self.sizes))

                if len(self.times) > self.window_size:
                    self.times.pop(0)
                    self.sizes.pop(0)
            except:
                traceback.print_exc()

    def print_bw(self):
        """print the average publishing rate to screen"""
        if len(self.times) < 2:
            return
        with self.lock:
            n = len(self.times)
            tn = time.time()
            t0 = self.times[0]

            total = sum(self.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n

            #std_dev = math.sqrt(sum((x - mean)**2 for x in self.sizes) /n)

            # min and max
            max_s = max(self.sizes)
            min_s = min(self.sizes)

        #min/max and even mean are likely to be much smaller, but for now I prefer unit consistency
        if bytes_per_s < 1000:
            bw, mean, min_s, max_s = ["%.2fB"%v for v in [bytes_per_s, mean, min_s, max_s]]
        elif bytes_per_s < 1000000:
            bw, mean, min_s, max_s = ["%.2fKB"%(v/1000) for v in [bytes_per_s, mean, min_s, max_s]]
        else:
            bw, mean, min_s, max_s = ["%.2fMB"%(v/1000000) for v in [bytes_per_s, mean, min_s, max_s]]

        print("average: %s/s\n\tmean: %s min: %s max: %s window: %s"%(bw, mean, min_s, max_s, n))

def _isstring_type(t):
    valid_types = [str]
    try:
        valid_types.append(unicode)
    except NameError:
        pass
    return t in valid_types

def _rostopic_bw(topic, window_size=-1):
    """
    periodically print the received bandwidth of a topic to console until
    shutdown
    """
    _check_master()
    _, real_topic, _ = get_topic_type(topic, blocking=True) #pause hz until topic is published
    if rospy.is_shutdown():
        return
    # #3543 disable all auto-subscriptions to /clock
    rospy.init_node(NAME, anonymous=True, disable_rostime=True)
    rt = ROSTopicBandwidth(window_size)
    # we use a large buffer size as we don't know what sort of messages we're dealing with.
    # may parameterize this in the future
    sub = rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback)
    print("subscribed to [%s]"%real_topic)
    while not rospy.is_shutdown():
        _sleep(1.0)
        rt.print_bw()

def _rostopic_cmd_bw(argv=sys.argv):
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog bw /topic", prog=NAME)
    parser.add_option("-w", "--window",
                      dest="window_size", default=None,
                      help="window size, in # of messages, for calculating rate", metavar="WINDOW")
    options, args = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    try:
        if options.window_size:
            import string
            window_size = string.atoi(options.window_size)
        else:
            window_size = options.window_size
    except:
        parser.error("window size must be an integer")
    topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    _rostopic_bw(topic, window_size=window_size)
