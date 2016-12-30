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

import rosgraph


_caller_apis = {}
def get_api(master, caller_id):
    """
    Get XML-RPC API of node
    :param master: XML-RPC handle to ROS Master, :class:`xmlrpclib.ServerProxy`
    :param caller_id: node name, ``str``
    :returns: XML-RPC URI of node, ``str``
    :raises: :exc:`ROSTopicIOException` If unable to communicate with master
    """
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        except rosgraph.MasterError:
            caller_api = 'unknown address %s'%caller_id

    return caller_api

def get_info_text(topic):
    """
    Get human-readable topic description

    :param topic: topic name, ``str``
    """
    try:
        from cStringIO import StringIO
    except ImportError:
        from io import StringIO
    import itertools
    buff = StringIO()
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master('/rostopic')
    try:
        state = master.getSystemState()

        pubs, subs, _ = state
        # filter based on topic
        subs = [x for x in subs if x[0] == topic]
        pubs = [x for x in pubs if x[0] == topic]

        topic_types = _master_get_topic_types(master)

    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    if not pubs and not subs:
        raise ROSTopicException("Unknown topic %s"%topic)

    buff.write("Type: %s\n\n"%topic_type(topic, topic_types))

    if pubs:
        buff.write("Publishers: \n")
        for p in itertools.chain(*[l for x, l in pubs]):
            buff.write(" * %s (%s)\n"%(p, get_api(master, p)))
    else:
        buff.write("Publishers: None\n")
    buff.write('\n')

    if subs:
        buff.write("Subscribers: \n")
        for p in itertools.chain(*[l for x, l in subs]):
            buff.write(" * %s (%s)\n"%(p, get_api(master, p)))
    else:
        buff.write("Subscribers: None\n")
    buff.write('\n')
    return buff.getvalue()

def _rostopic_info(topic):
    """
    Print topic information to screen.

    :param topic: topic name, ``str``
    """
    print(get_info_text(topic))

def _rostopic_cmd_info(argv):
    """
    Command-line parsing for 'rostopic info' command.
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog info /topic", prog=NAME)
    (options, args) = parser.parse_args(args)

    if len(args) == 0:
        parser.error("you must specify a topic name")
    elif len(args) > 1:
        parser.error("you may only specify one topic name")

    topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    exitval = _rostopic_info(topic) or 0
    if exitval != 0:
        sys.exit(exitval)
