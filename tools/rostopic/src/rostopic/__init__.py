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

NAME='rostopic'

import os
import sys
import math
import socket
import time
try:
    from xmlrpc.client import Fault
except ImportError:
    from xmlrpclib import Fault

import rosgraph
import rospy

from rostopic.exceptions import ROSTopicException


def _optparse_topic_only(cmd, argv):
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %%prog %s /topic"%cmd, prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    return rosgraph.names.script_resolve_name('rostopic', args[0])


def _fullusage():
    print("""rostopic is a command-line tool for printing information about ROS Topics.

Commands:
\trostopic bw\tdisplay bandwidth used by topic
\trostopic delay\tdisplay delay of topic from timestamp in header
\trostopic echo\tprint messages to screen
\trostopic find\tfind topics by type
\trostopic hz\tdisplay publishing rate of topic
\trostopic info\tprint information about active topic
\trostopic list\tlist active topics
\trostopic pub\tpublish data to topic
\trostopic type\tprint topic or field type

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))


from rostopic.verbs.echo import _rostopic_cmd_echo
from rostopic.verbs.hz import _rostopic_cmd_hz
from rostopic.verbs.type import _rostopic_cmd_type
from rostopic.verbs.list import _rostopic_cmd_list
from rostopic.verbs.info import _rostopic_cmd_info
from rostopic.verbs.pub import _rostopic_cmd_pub
from rostopic.verbs.bw import _rostopic_cmd_bw
from rostopic.verbs.find import _rostopic_cmd_find
from rostopic.verbs.delay import _rostopic_cmd_delay


def rostopicmain(argv=None):
    import rosbag
    if argv is None:
        argv=sys.argv
    # filter out remapping arguments in case we are being invoked via roslaunch
    argv = rospy.myargv(argv)

    # process argv
    if len(argv) == 1:
        _fullusage()
    try:
        command = argv[1]
        if command == 'echo':
            _rostopic_cmd_echo(argv)
        elif command == 'hz':
            _rostopic_cmd_hz(argv)
        elif command == 'type':
            _rostopic_cmd_type(argv)
        elif command == 'list':
            _rostopic_cmd_list(argv)
        elif command == 'info':
            _rostopic_cmd_info(argv)
        elif command == 'pub':
            _rostopic_cmd_pub(argv)
        elif command == 'bw':
            _rostopic_cmd_bw(argv)
        elif command == 'find':
            _rostopic_cmd_find(argv)
        elif command == 'delay':
            _rostopic_cmd_delay(argv)
        else:
            _fullusage()
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")
        sys.exit(1)
    except rosbag.ROSBagException as e:
        sys.stderr.write("ERROR: unable to use bag file: %s\n"%str(e))
        sys.exit(1)
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)
    except ROSTopicException as e:
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)
    except KeyboardInterrupt: pass
    except rospy.ROSInterruptException: pass
