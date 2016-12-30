
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

import argparse
import rospy


class ROSTopicDelay(object):

    def __init__(self, window_size):
        import threading
        self.lock = threading.Lock()
        self.last_msg_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.delays = []

        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

    def callback_delay(self, msg):
        if not msg._has_header:
            rospy.logerr('msg does not have header')
            return
        with self.lock:
            curr_rostime = rospy.get_rostime()

            # time reset
            if curr_rostime.is_zero():
                if len(self.delays) > 0:
                    print("time has reset, resetting counters")
                    self.delays = []
                return

            curr = curr_rostime.to_sec()
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.delays = []
            else:
                self.delays.append(curr_rostime.to_time() -
                                   msg.header.stamp.to_time())
                self.msg_tn = curr

            if len(self.delays) > self.window_size - 1:
                self.delays.pop(0)

    def get_delay(self):
        if self.msg_tn == self.last_msg_tn:
            return
        with self.lock:
            if not self.delays:
                return
            n = len(self.delays)

            mean = sum(self.delays) / n
            rate = 1. / mean if mean > 0 else 0

            std_dev = math.sqrt(sum((x - mean)**2 for x in self.delays) / n)

            max_delta = max(self.delays)
            min_delta = min(self.delays)

            self.last_msg_tn = self.msg_tn
        return mean, min_delta, max_delta, std_dev, n + 1

    def print_delay(self):
        """
        print the average publishing delay to screen
        """
        if not self.delays:
            return
        ret = self.get_delay()
        if ret is None:
            print("no new messages")
            return
        delay, min_delta, max_delta, std_dev, window = ret
        print("average delay: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(delay, min_delta, max_delta, std_dev, window))


def _rostopic_delay(topic, window_size=-1):
    """
    Periodically print the publishing delay of a topic to console until
    shutdown
    :param topic: topic name, ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    """
    # pause hz until topic is published
    msg_class, real_topic, _ = get_topic_class(topic, blocking=True)
    if rospy.is_shutdown():
        return
    rospy.init_node(NAME, anonymous=True)
    rt = ROSTopicDelay(window_size)
    sub = rospy.Subscriber(real_topic, msg_class, rt.callback_delay)
    print("subscribed to [%s]" % real_topic)

    if rospy.get_param('use_sim_time', False):
        print("WARNING: may be using simulated time",file=sys.stderr)

    while not rospy.is_shutdown():
        _sleep(1.0)
        rt.print_delay()


def _rostopic_cmd_delay(argv):
    args = argv[2:]
    import argparse
    parser = argparse.ArgumentParser(usage="%(prog)s delay [options] /topic", prog=NAME)
    parser.add_argument("topic", help="topic name to be calcurated the delay")
    parser.add_argument("-w", "--window",
                        dest="window_size", default=-1, type=int,
                        help="window size, in # of messages, for calculating rate")

    args = parser.parse_args(args)
    topic_name = args.topic
    window_size = args.window_size
    topic = rosgraph.names.script_resolve_name('rostopic', topic_name)
    _rostopic_delay(topic, window_size=window_size)
