#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Victor Lamoine - Institut Maupertuis
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

from __future__ import print_function

import argparse
import datetime
import os
import rospy
import rosbag

if __name__ == '__main__':
    import sys
    parser = argparse.ArgumentParser(description='Trim bag files.')
    parser.add_argument('inbag_filename', type=str, help='the input bag file')
    parser.add_argument('outbag_filename', type=str, help='the output bag file')
    parser.add_argument('begin_trim_time', type=float, help='the begin trim time (in seconds, relative to the start of the bag file)')
    parser.add_argument('end_trim_time', type=float, help='the begin trim time (in seconds, relative to the start of the bag file)')
    args = parser.parse_args()

    inbag_filename = args.inbag_filename
    outbag_filename = args.outbag_filename
    begin_trim = args.begin_trim_time
    end_trim = args.end_trim_time

    if begin_trim >= end_trim:
        print("Begin trim time cannot be >= to end trim time", file=sys.stderr)
        sys.exit(1)

    if os.path.realpath(inbag_filename) == os.path.realpath(outbag_filename):
        print('Cannot use same file as input and output [%s]' % inbag_filename, file=sys.stderr)
        sys.exit(2)

    try:
        inbag = rosbag.Bag(inbag_filename, 'r')
    except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
        print('ERROR: %s' % str(ex), file=sys.stderr)
        sys.exit(3)
    except ROSBagUnindexedException as ex:
        print('ERROR bag unindexed: %s.  Run rosbag reindex.' % inbag_filename, file=sys.stderr)
        sys.exit(3)

    start = inbag._get_yaml_info("start")
    if start is None:
        print ("Could not get start time of bag file", file=sys.stderr)
        sys.exit(4)
    start = float(start)
    print("Start time", start, "which was", datetime.datetime.fromtimestamp(start))

    end = inbag._get_yaml_info("end")
    if start is None:
        print("Could not get end time of bag file", file=sys.stderr)
        sys.exit(4)
    end = float(end)
    print("End time  ", end, "which was", datetime.datetime.fromtimestamp(end))

    print("Data before", begin_trim, "seconds and after", end_trim, "seconds will be trimmed")
    #rosbag.filter_cmd()

    outbag = rosbag.Bag(outbag_filename, 'w')
    # FIXME Trim bag file...
    # https://github.com/ros/ros_comm/blob/melodic-devel/tools/rosbag/src/rosbag/rosbag_main.py#L301-L387

    inbag.close()
    outbag.close()
