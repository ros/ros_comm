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
import rospy


def _rostopic_list_bag(bag_file, topic=None):
    """
    Prints topics in bag file to screen
    :param bag_file: path to bag file, ``str``
    :param topic: optional topic name to match. Will print additional information just about messagese in this topic, ``str``
    """
    import rosbag
    if not os.path.exists(bag_file):
        raise ROSTopicException("bag file [%s] does not exist"%bag_file)

    with rosbag.Bag(bag_file) as b:
        if topic:
            # create string for namespace comparison
            topic_ns = rosgraph.names.make_global_ns(topic)
            count = 0
            earliest = None
            latest = None
            for top, msg, t in b.read_messages(raw=True):
                if top == topic or top.startswith(topic_ns):
                    count += 1
                    if earliest == None:
                        earliest = t

                    latest = t
                if rospy.is_shutdown():
                    break
            import time
            earliest, latest = [time.strftime("%d %b %Y %H:%M:%S", time.localtime(t.to_time())) for t in (earliest, latest)]
            print("%s message(s) from %s to %s"%(count, earliest, latest))
        else:
            topics = set()
            for top, msg, _ in b.read_messages(raw=True):
                if top not in topics:
                    print(top)
                    topics.add(top)
                if rospy.is_shutdown():
                    break

def _sub_rostopic_list(master, pubs, subs, publishers_only, subscribers_only, verbose, indent=''):
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    if verbose:
        topic_types = _master_get_topic_types(master)

        if not subscribers_only:
            print("\n%sPublished topics:"%indent)
            for t, l in pubs:
                if len(l) > 1:
                    print(indent+" * %s [%s] %s publishers"%(t, topic_type(t, topic_types), len(l)))
                else:
                    print(indent+" * %s [%s] 1 publisher"%(t, topic_type(t, topic_types)))

        if not publishers_only:
            print(indent)
            print(indent+"Subscribed topics:")
            for t,l in subs:
                if len(l) > 1:
                    print(indent+" * %s [%s] %s subscribers"%(t, topic_type(t, topic_types), len(l)))
                else:
                    print(indent+" * %s [%s] 1 subscriber"%(t, topic_type(t, topic_types)))
        print('')
    else:
        if publishers_only:
            topics = [t for t,_ in pubs]
        elif subscribers_only:
            topics = [t for t,_ in subs]
        else:
            topics = list(set([t for t,_ in pubs] + [t for t,_ in subs]))
        topics.sort()
        print('\n'.join(["%s%s"%(indent, t) for t in topics]))

# #3145
def _rostopic_list_group_by_host(master, pubs, subs):
    """
    Build up maps for hostname to topic list per hostname
    :returns: publishers host map, subscribers host map, ``{str: set(str)}, {str: set(str)}``
    """
    def build_map(master, state, uricache):
        tmap = {}
        for topic, tnodes in state:
            for p in tnodes:
                if not p in uricache:
                   uricache[p] = master.lookupNode(p)
                uri = uricache[p]
                puri = urlparse(uri)
                if not puri.hostname in tmap:
                    tmap[puri.hostname] = []
                # recreate the system state data structure, but for a single host
                matches = [l for x, l in tmap[puri.hostname] if x == topic]
                if matches:
                    matches[0].append(p)
                else:
                    tmap[puri.hostname].append((topic, [p]))
        return tmap

    uricache = {}
    host_pub_topics = build_map(master, pubs, uricache)
    host_sub_topics = build_map(master, subs, uricache)
    return host_pub_topics, host_sub_topics

def _rostopic_list(topic, verbose=False,
                   subscribers_only=False, publishers_only=False,
                   group_by_host=False):
    """
    Print topics to screen

    :param topic: topic name to list information or None to match all topics, ``str``
    :param verbose: print additional debugging information, ``bool``
    :param subscribers_only: print information about subscriptions only, ``bool``
    :param publishers_only: print information about subscriptions only, ``bool``
    :param group_by_host: group topic list by hostname, ``bool``
    """
    # #1563
    if subscribers_only and publishers_only:
        raise ROSTopicException("cannot specify both subscribers- and publishers-only")

    master = rosgraph.Master('/rostopic')
    try:
        state = master.getSystemState()

        pubs, subs, _ = state
        if topic:
            # filter based on topic
            topic_ns = rosgraph.names.make_global_ns(topic)
            subs = (x for x in subs if x[0] == topic or x[0].startswith(topic_ns))
            pubs = (x for x in pubs if x[0] == topic or x[0].startswith(topic_ns))

    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    if group_by_host:
        # #3145
        host_pub_topics, host_sub_topics  = _rostopic_list_group_by_host(master, pubs, subs)
        for hostname in set(list(host_pub_topics.keys()) + list(host_sub_topics.keys())): #py3k
            pubs, subs = host_pub_topics.get(hostname,[]), host_sub_topics.get(hostname, []),
            if (pubs and not subscribers_only) or (subs and not publishers_only):
                print("Host [%s]:" % hostname)
                _sub_rostopic_list(master, pubs, subs,
                                   publishers_only, subscribers_only,
                                   verbose, indent='  ')
    else:
        _sub_rostopic_list(master, pubs, subs,
                           publishers_only, subscribers_only,
                           verbose)

def _rostopic_cmd_list(argv):
    """
    Command-line parsing for 'rostopic list' command.
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog list [/namespace]", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="list topics in .bag file", metavar="BAGFILE")
    parser.add_option("-v", "--verbose",
                      dest="verbose", default=False,action="store_true",
                      help="list full details about each topic")
    parser.add_option("-p",
                      dest="publishers", default=False,action="store_true",
                      help="list only publishers")
    parser.add_option("-s",
                      dest="subscribers", default=False,action="store_true",
                      help="list only subscribers")
    parser.add_option("--host", dest="hostname", default=False, action="store_true",
                      help="group by host name")

    (options, args) = parser.parse_args(args)
    topic = None

    if len(args) == 1:
        topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    elif len(args) > 1:
        parser.error("you may only specify one input topic")
    if options.bag:
        if options.subscribers:
            parser.error("-s option is not valid with bags")
        elif options.publishers:
            parser.error("-p option is not valid with bags")
        elif options.hostname:
            parser.error("--host option is not valid with bags")
        _rostopic_list_bag(options.bag, topic)
    else:
        if options.subscribers and options.publishers:
            parser.error("you may only specify one of -p, -s")

        exitval = _rostopic_list(topic, verbose=options.verbose, subscribers_only=options.subscribers, publishers_only=options.publishers, group_by_host=options.hostname) or 0
        if exitval != 0:
            sys.exit(exitval)
