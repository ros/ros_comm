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

from argparse import ArgumentParser
from rosgraph.names import script_resolve_name
from rostopic import NAME
from rostopic.util import get_topic_type


def _rostopic_type(topic):
    """
    Print ROS message type of topic to screen
    :param topic: topic name, ``str``
    """
    topic_type, topic_real_name, _ = get_topic_type(topic, blocking=False)
    if topic_type is None:
        sys.stderr.write('unknown topic type [%s]\n'%topic)
        sys.exit(1)
    elif topic == topic_real_name:
        print(topic_type)
    else:
        field = topic[len(topic_real_name)+1:]
        field_type = topic_type
        for current_field in field.split('/'):
            msg_class = roslib.message.get_message_class(field_type)
            field_type = msg_class._slot_types[msg_class.__slots__.index(current_field)]
        print('%s %s %s'%(topic_type, field, field_type))

def _rostopic_cmd_type(argv):
    parser = ArgumentParser(prog='%s type' % NAME)
    parser.add_argument('topic_or_field', help='Topic or field name')
    args = parser.parse_args(argv[2:])
    _rostopic_type(script_resolve_name('rostopic', args.topic_or_field))
