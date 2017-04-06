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

"""
rosnode implements the rosnode command-line tool and also provides a
library for retrieving ROS Node information.
"""

from __future__ import print_function

import os
import errno
import sys
import socket
import time

try: #py3k
    import urllib.parse as urlparse
except ImportError:
    import urlparse

from optparse import OptionParser
from rosgraph.xmlrpc import ServerProxy
import rosgraph
import rosgraph.names
import rostopic

NAME='rosnode'
ID = '/rosnode'

class ROSNodeException(Exception):
    """
    rosnode base exception type
    """
    pass
class ROSNodeIOException(ROSNodeException):
    """
    Exceptions for communication-related (i/o) errors, generally due to Master or Node network communication issues.
    """
    pass

# need for calling node APIs
def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val

_caller_apis = {}
def get_api_uri(master, caller_id, skip_cache=False):
    """
    @param master: XMLRPC handle to ROS Master
    @type  master: rosgraph.Master
    @param caller_id: node name
    @type  caller_id: str
    @param skip_cache: flag to skip cached data and force to lookup node from master
    @type  skip_cache: bool
    @return: xmlrpc URI of caller_id
    @rtype: str
    @raise ROSNodeIOException: if unable to communicate with master
    """
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api or skip_cache:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except rosgraph.MasterError:
            return None
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
    return caller_api

def lookup_uri(master, system_state, topic, uri):
    for l in system_state[0:2]:
        for entry in l:
            if entry[0] == topic:
                for n in entry[1]:
                    if rostopic.get_api(master, n) == uri:
                        return '%s (%s)' % (n, uri)
    return uri

def get_node_names(namespace=None):
    """
    @param namespace: optional namespace to scope return values by. Namespace must already be resolved.
    @type  namespace: str
    @return: list of node caller IDs
    @rtype: [str]
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    nodes = []
    if namespace:
        # canonicalize namespace with leading/trailing slash
        g_ns = rosgraph.names.make_global_ns(namespace)
        for s in state:
            for t, l in s:
                nodes.extend([n for n in l if n.startswith(g_ns) or n == namespace])
    else:
        for s in state:
            for t, l in s:
                nodes.extend(l)
    return list(set(nodes))

def get_machines_by_nodes():
    """
    Find machines connected to nodes. This is a very costly procedure as it
    must do N lookups with the Master, where N is the number of nodes.
    
    @return: list of machines connected
    @rtype: [str]
    @raise ROSNodeIOException: if unable to communicate with master
    """

    master = rosgraph.Master(ID)

    # get all the node names, lookup their uris, parse the hostname
    # from the uris, and then compare the resolved hostname against
    # the requested machine name.
    machines = []
    node_names = get_node_names()
    for n in node_names:
        try:
            uri = master.lookupNode(n)
            h = urlparse.urlparse(uri).hostname
            if h not in machines:
                machines.append(h)

        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")    
        except rosgraph.MasterError:
            # it's possible that the state changes as we are doing lookups. this is a soft-fail
            continue
    return machines


def get_nodes_by_machine(machine):
    """
    Find nodes by machine name. This is a very costly procedure as it
    must do N lookups with the Master, where N is the number of nodes.
    
    @return: list of nodes on the specified machine
    @rtype: [str]
    @raise ROSNodeException: if machine name cannot be resolved to an address
    @raise ROSNodeIOException: if unable to communicate with master
    """
    import urlparse
    
    master = rosgraph.Master(ID)
    try:
        machine_actual = [host[4][0] for host in socket.getaddrinfo(machine, 0, 0, 0, socket.SOL_TCP)]
    except:
        raise ROSNodeException("cannot resolve machine name [%s] to address"%machine)

    # get all the node names, lookup their uris, parse the hostname
    # from the uris, and then compare the resolved hostname against
    # the requested machine name.
    matches = [machine] + machine_actual
    not_matches = [] # cache lookups
    node_names = get_node_names()
    retval = []
    for n in node_names:
        try:
            try:
                uri = master.lookupNode(n)
            except rosgraph.MasterError:
                # it's possible that the state changes as we are doing lookups. this is a soft-fail
                continue

            h = urlparse.urlparse(uri).hostname
            if h in matches:
                retval.append(n)
            elif h in not_matches:
                continue
            else:
                r = [host[4][0] for host in socket.getaddrinfo(h, 0, 0, 0, socket.SOL_TCP)]
                if set(r) & set(machine_actual):
                    matches.append(r)
                    retval.append(n)
                else:
                    not_matches.append(r)                        
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")    
    return retval

def kill_nodes(node_names):
    """
    Call shutdown on the specified nodes

    @return: list of nodes that shutdown was called on successfully and list of failures
    @rtype: ([str], [str])
    """
    master = rosgraph.Master(ID)
    
    success = []
    fail = []
    tocall = []
    try:
        # lookup all nodes keeping track of lookup failures for return value
        for n in node_names:
            try:
                uri = master.lookupNode(n)
                tocall.append([n, uri])
            except:
                fail.append(n)
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    for n, uri in tocall:
        # the shutdown call can sometimes fail to succeed if the node
        # tears down during the request handling, so we assume success
        try:
            p = ServerProxy(uri)
            _succeed(p.shutdown(ID, 'user request'))
        except:
            pass
        success.append(n)            

    return success, fail

def _sub_rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    """
    Subroutine for rosnode_listnodes(). Composes list of strings to print to screen.
    
    @param namespace: (default None) namespace to scope list to. 
    @type  namespace: str
    @param list_uri: (default False) return uris of nodes instead of names. 
    @type  list_uri: bool
    @param list_all: (default False) return names and uris of nodes as combined strings
    @type  list_all: bool
    @return: new-line separated string containing list of all nodes
    @rtype: str
    """
    master = rosgraph.Master(ID)
    nodes = get_node_names(namespace)
    nodes.sort()
    if list_all:
        return '\n'.join(["%s \t%s"%(get_api_uri(master, n) or 'unknown address', n) for n in nodes])
    elif list_uri:
        return '\n'.join([(get_api_uri(master, n) or 'unknown address') for n in nodes])
    else:
        return '\n'.join(nodes)
    
def rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    """
    Print list of all ROS nodes to screen.

    @param namespace: namespace to scope list to
    @type  namespace: str
    @param list_uri: print uris of nodes instead of names
    @type  list_uri: bool
    @param list_all: print node names and uris
    @param list_all: bool    
    """
    print(_sub_rosnode_listnodes(namespace=namespace, list_uri=list_uri, list_all=list_all))
    
def rosnode_ping(node_name, max_count=None, verbose=False):
    """
    Test connectivity to node by calling its XMLRPC API
    @param node_name: name of node to ping
    @type  node_name: str
    @param max_count: number of ping requests to make
    @type  max_count: int
    @param verbose: print ping information to screen
    @type  verbose: bool
    @return: True if node pinged
    @rtype: bool
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    node_api = get_api_uri(master,node_name)
    if not node_api:
        print("cannot ping [%s]: unknown node"%node_name, file=sys.stderr)
        return False

    timeout = 3.

    if verbose:
        print("pinging %s with a timeout of %ss"%(node_name, timeout))
    socket.setdefaulttimeout(timeout)
    node = ServerProxy(node_api)
    lastcall = 0.
    count = 0
    acc = 0.
    try:
        while True:
            try:
                start = time.time()
                pid = _succeed(node.getPid(ID))
                end = time.time()

                dur = (end-start)*1000.
                acc += dur
                count += 1

                if verbose:
                    print("xmlrpc reply from %s\ttime=%fms"%(node_api, dur))
                # 1s between pings
            except socket.error as e:
                # 3786: catch ValueError on unpack as socket.error is not always a tuple
                try:
                    # #3659
                    errnum, msg = e
                    if errnum == -2: #name/service unknown
                        p = urlparse.urlparse(node_api)
                        print("ERROR: Unknown host [%s] for node [%s]"%(p.hostname, node_name), file=sys.stderr)
                    elif errnum == errno.ECONNREFUSED:
                        # check if node url has changed
                        new_node_api = get_api_uri(master,node_name, skip_cache=True)
                        if not new_node_api:
                            print("cannot ping [%s]: unknown node"%node_name, file=sys.stderr)
                            return False
                        if new_node_api != node_api:
                            if verbose:
                                print("node url has changed from [%s] to [%s], retrying to ping"%(node_api, new_node_api))
                            node_api = new_node_api
                            node = ServerProxy(node_api)
                            continue
                        print("ERROR: connection refused to [%s]"%(node_api), file=sys.stderr)
                    else:
                        print("connection to [%s] timed out"%node_name, file=sys.stderr)
                    return False
                except ValueError:
                    print("unknown network error contacting node: %s"%(str(e)))
            if max_count and count >= max_count:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
            
    if verbose and count > 1:
        print("ping average: %fms"%(acc/count))
    return True

def rosnode_ping_all(verbose=False):
    """
    Ping all running nodes
    @return [str], [str]: pinged nodes, un-pingable nodes
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    nodes = []
    for s in state:
        for t, l in s:
            nodes.extend(l)
    nodes = list(set(nodes)) #uniq
    if verbose:
        print("Will ping the following nodes: \n"+''.join([" * %s\n"%n for n in nodes]))
    pinged = []
    unpinged = []
    for node in nodes:
        if rosnode_ping(node, max_count=1, verbose=verbose):
            pinged.append(node)
        else:
            unpinged.append(node)
    return pinged, unpinged
    
def cleanup_master_blacklist(master, blacklist):
    """
    Remove registrations from ROS Master that do not match blacklist.    
    @param master: XMLRPC handle to ROS Master
    @type  master: rosgraph.xmlrpc.ServerProxy
    @param blacklist: list of nodes to scrub
    @type  blacklist: [str]
    """
    pubs, subs, srvs = master.getSystemState()
    for n in blacklist:
        print("Unregistering", n)
        node_api = get_api_uri(master, n)
        for t, l in pubs:
            if n in l:
                master_n = rosgraph.Master(n)
                master_n.unregisterPublisher(t, node_api)
        for t, l in subs:
            if n in l:
                master_n = rosgraph.Master(n)
                master_n.unregisterSubscriber(t, node_api)
        for s, l in srvs:
            if n in l:
                service_api = master.lookupService(s)
                master_n = rosgraph.Master(n)
                master_n.unregisterService(s, service_api)

def cleanup_master_whitelist(master, whitelist):
    """
    Remove registrations from ROS Master that do not match whitelist.
    @param master: XMLRPC handle to ROS Master
    @type  master: nosgraph.xmlrpc.ServerProxy
    @param whitelist: list of nodes to keep
    @type  whitelist: list of nodes to keep
   """
    pubs, subs, srvs = master.getSystemState()
    for t, l in pubs:
        for n in l:
            if n not in whitelist:
                node_api = get_api_uri(master, n)
                master_n = rosgraph.Master(n)
                master_n.unregisterPublisher(t, node_api)
    for t, l in subs:
        for n in l:
            if n not in whitelist:
                node_api = get_api_uri(master, n)
                master_n = rosgraph.Master(n)                
                master_n.unregisterSubscriber(t, node_api)
    for s, l in srvs:
        for n in l:
            if n not in whitelist:
                service_api = master.lookupService(s)
                master_n = rosgraph.Master(n)
                master_n.unregisterService(s, service_api)

def rosnode_cleanup():
    """
    This is a semi-hidden routine for cleaning up stale node
    registration information on the ROS Master. The intent is to
    remove this method once Master TTLs are properly implemented.
    """
    pinged, unpinged = rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(ID)
        print("Unable to contact the following nodes:")
        print('\n'.join(' * %s'%n for n in unpinged))
        print("Warning: these might include alive and functioning nodes, e.g. in unstable networks.")
        print("Cleanup will purge all information about these nodes from the master.")
        print("Please type y or n to continue:")
        input = sys.stdin.readline()
        while not input.strip() in ['y', 'n']:
            input = sys.stdin.readline()
        if input.strip() == 'n':
            print('aborting')
            return
        
        cleanup_master_blacklist(master, unpinged)

        print("done")

def get_node_info_description(node_name):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master(ID)

    # go through the master system state first
    try:
        state = master.getSystemState()
        pub_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    pubs = [t for t, l in state[0] if node_name in l]
    subs = [t for t, l in state[1] if node_name in l]
    srvs = [t for t, l in state[2] if node_name in l]  

    buff = "Node [%s]"%node_name
    if pubs:
        buff += "\nPublications: \n"
        buff += '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in pubs]) + '\n'
    else:
        buff += "\nPublications: None\n"
    if subs:
        buff += "\nSubscriptions: \n"
        buff += '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in subs]) + '\n'
    else:
        buff += "\nSubscriptions: None\n"        
    if srvs:
        buff += "\nServices: \n"
        buff += '\n'.join([" * %s"%l for l in srvs]) + '\n'
    else:
        buff += "\nServices: None\n"
        
    return buff

def get_node_connection_info_description(node_api, master):
    #turn down timeout on socket library
    socket.setdefaulttimeout(5.0)
    node = ServerProxy(node_api)
    system_state = master.getSystemState()

    try:
        pid = _succeed(node.getPid(ID))
        buff = "Pid: %s\n"%pid
        #master_uri = _succeed(node.getMasterUri(ID))
        businfo = _succeed(node.getBusInfo(ID))
        if businfo:
            buff += "Connections:\n"
            for info in businfo:
                dest_id   = info[1]
                direction = info[2]
                transport = info[3]
                topic     = info[4]
                if len(info) > 5:
                    connected = info[5]
                else:
                    connected = True #backwards compatibility

                if connected:
                    buff += " * topic: %s\n"%topic

                    # older ros publisher implementations don't report a URI
                    buff += "    * to: %s\n"%lookup_uri(master, system_state, topic, dest_id)
                    if direction == 'i':
                        buff += "    * direction: inbound\n"
                    elif direction == 'o':
                        buff += "    * direction: outbound\n"
                    else:
                        buff += "    * direction: unknown\n"
                    buff += "    * transport: %s\n"%transport
    except socket.error:
        raise ROSNodeIOException("Communication with node[%s] failed!"%(node_api))
    return buff

def rosnode_info(node_name, quiet=False):
    """
    Print information about node, including subscriptions and other debugging information. This will query the node over the network.
    
    @param node_name: name of ROS node
    @type  node_name: str
    @raise ROSNodeIOException: if unable to communicate with master
    """
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master(ID)
    node_name = rosgraph.names.script_resolve_name('rosnode', node_name)

    print('-'*80)
    print(get_node_info_description(node_name))
        
    node_api = get_api_uri(master, node_name)
    if not node_api:
        print("cannot contact [%s]: unknown node"%node_name, file=sys.stderr)
        return

    if not quiet:
        print("\ncontacting node %s ..." % node_api)
        print(get_node_connection_info_description(node_api, master))

# backwards compatibility (deprecated)
rosnode_debugnode = rosnode_info

def _rosnode_cmd_list(argv):
    """
    Implements rosnode 'list' command.
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog list", prog=NAME)
    parser.add_option("-u",
                      dest="list_uri", default=False,
                      action="store_true",
                      help="list XML-RPC URIs")
    parser.add_option("-a","--all",
                      dest="list_all", default=False,
                      action="store_true",
                      help="list all information")
    (options, args) = parser.parse_args(args)
    namespace = None
    if len(args) > 1:
        parser.error("invalid args: you may only specify one namespace")
    elif len(args) == 1:
        namespace = rosgraph.names.script_resolve_name('rostopic', args[0])
    rosnode_listnodes(namespace=namespace, list_uri=options.list_uri, list_all=options.list_all)

def _rosnode_cmd_info(argv):
    """
    Implements rosnode 'info' command.
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog info [options] node1 [node2...]",
                          prog=NAME)
    parser.add_option("-q", "--quiet",
                      dest="quiet", default=False,
                      action="store_true",
                      help="Prints only basic information such as pubs/subs and does not contact nodes for more information")
    (options, args) = parser.parse_args(args)
    if not args:
        parser.error("You must specify at least one node name")        
    for node in args:
        rosnode_info(node, options.quiet)

def _rosnode_cmd_machine(argv):
    """
    Implements rosnode 'machine' command.

    @raise ROSNodeException: if user enters in unrecognized machine name
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog machine [machine-name]", prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) > 1:
        parser.error("please enter only one machine name")
    elif len(args) == 0:
        machines = get_machines_by_nodes()
        machines.sort()
        print('\n'.join(machines))
    else:
        nodes = get_nodes_by_machine(args[0])
        nodes.sort()
        print('\n'.join(nodes))

def _rosnode_cmd_kill(argv):
    """
    Implements rosnode 'kill' command.

    @raise ROSNodeException: if user enters in unrecognized nodes
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog kill [node]...", prog=NAME)
    parser.add_option("-a","--all",
                      dest="kill_all", default=False,
                      action="store_true",
                      help="kill all nodes")

    (options, args) = parser.parse_args(args)
    if options.kill_all:
        if args:
            parser.error("invalid arguments with kill all (-a) option")
        args = get_node_names()
        args.sort()
    elif not args:
        node_list = get_node_names()
        node_list.sort()
        if not node_list:
            print("No nodes running", file=sys.stderr)
            return 0
        
        sys.stdout.write('\n'.join(["%s. %s"%(i+1, n) for i,n in enumerate(node_list)]))
        sys.stdout.write("\n\nPlease enter the number of the node you wish to kill.\n> ")
        selection = ''
        while not selection:
            selection = sys.stdin.readline().strip()
            try:
                selection = int(selection) 
                if selection <= 0:
                    print("ERROR: invalid selection. Please enter a number (ctrl-C to cancel)")
            except:
                print("ERROR: please enter a number (ctrl-C to cancel)")
                sys.stdout.flush()
                selection = ''
        args = [node_list[selection - 1]]
    else:
        # validate args
        args = [rosgraph.names.script_resolve_name(ID, n) for n in args]
        node_list = get_node_names()
        unknown = [n for n in args if not n in node_list]
        if unknown:
            raise ROSNodeException("Unknown node(s):\n"+'\n'.join([" * %s"%n for n in unknown]))
    if len(args) > 1:
        print("killing:\n"+'\n'.join([" * %s"%n for n in args]))
    else:
        print("killing %s"%(args[0]))
            
    success, fail = kill_nodes(args)
    if fail:
        print("ERROR: Failed to kill:\n"+'\n'.join([" * %s"%n for n in fail]), file=sys.stderr)
        return 1
    print("killed")
    return 0
        
def _rosnode_cmd_cleanup(argv):
    """
    Implements rosnode 'cleanup' command.
    @param argv: command-line args
    @type  argv: [str]
    """
    args = argv[2:]
    parser = OptionParser(usage="usage: %prog cleanup", prog=NAME)
    (options, args) = parser.parse_args(args)
    rosnode_cleanup()

def _rosnode_cmd_ping(argv):
    """
    Implements rosnode 'ping' command.
    @param argv: command-line args
    @type  argv: [str]
    """
    args = argv[2:]    
    parser = OptionParser(usage="usage: %prog ping [options] <node>", prog=NAME)
    parser.add_option("--all", "-a",
                      dest="ping_all", default=False,
                      action="store_true",
                      help="ping all nodes")
    parser.add_option("-c",
                      dest="ping_count", default=None, metavar="COUNT",type="int",
                      help="number of pings to send. Not available with --all")
    (options, args) = parser.parse_args(args)

    node_name = None
    if not options.ping_all:
        if not args:
            try:
                parser.error("Please enter a node to ping. Available nodes are:\n"+_sub_rosnode_listnodes())
            except:
                # master is offline, but user did have invalid usage, so display correct usage first
                parser.error("Please enter a node to ping")
        elif len(args) > 1:
            parser.error("you may only specify one input node")
        elif len(args) == 1:
            node_name = rosgraph.names.script_resolve_name('rosnode', args[0])
            print("rosnode: node is [%s]"%node_name)
    else:
        if args:
            parser.error("Invalid arguments '%s' used with --all"%(' '.join(args)))
        elif options.ping_count:
            parser.error("-c may not be used with --all")
            
    if options.ping_all:
        rosnode_ping_all(verbose=True)
    else:
        rosnode_ping(node_name, verbose=True, max_count=options.ping_count)
    
def _fullusage(return_error=True):
    """
    Prints rosnode usage information.
    @param return_error whether to exit with error code os.EX_USAGE
    """
    print("""rosnode is a command-line tool for printing information about ROS Nodes.

Commands:
\trosnode ping\ttest connectivity to node
\trosnode list\tlist active nodes
\trosnode info\tprint information about node
\trosnode machine\tlist nodes running on a particular machine or list machines
\trosnode kill\tkill a running node
\trosnode cleanup\tpurge registration information of unreachable nodes

Type rosnode <command> -h for more detailed usage, e.g. 'rosnode ping -h'
""")
    if return_error:
        sys.exit(getattr(os, 'EX_USAGE', 1))
    else:
        sys.exit(0)

def rosnodemain(argv=None):
    """
    Prints rosnode main entrypoint.
    @param argv: override sys.argv
    @param argv: [str]
    """
    if argv == None:
        argv = sys.argv
    if len(argv) == 1:
        _fullusage()
    try:
        command = argv[1]
        if command == 'ping':
            sys.exit(_rosnode_cmd_ping(argv) or 0)
        elif command == 'list':
            sys.exit(_rosnode_cmd_list(argv) or 0)
        elif command == 'info':
            sys.exit(_rosnode_cmd_info(argv) or 0)
        elif command == 'machine':
            sys.exit(_rosnode_cmd_machine(argv) or 0)
        elif command == 'cleanup':
            sys.exit(_rosnode_cmd_cleanup(argv) or 0)
        elif command == 'kill':
            sys.exit(_rosnode_cmd_kill(argv) or 0)
        elif command == '--help':
            _fullusage(False)
        else:
            _fullusage()
    except socket.error:
        print("Network communication failed. Most likely failed to communicate with master.", file=sys.stderr)
    except rosgraph.MasterError as e:
        print("ERROR: "+str(e), file=sys.stderr)
    except ROSNodeException as e:
        print("ERROR: "+str(e), file=sys.stderr)
    except KeyboardInterrupt:
        pass
