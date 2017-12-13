#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, SRI International
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


import argparse
import os
import sys
import argparse 
import xmlrpclib
import sys 
from rosxmlrpc import ROSXmlRpc


def get_node_uri( node, master_uri, caller_id = "anon" ):
  rosxmlrpc = ROSXmlRpc( master_uri, caller_id )
  ret, msg, node_uri = rosxmlrpc.call( "lookupNode", [node] )
  return node_uri if ret > 0 else ""

""" Utility script to call XMLRPC commands on nodes.
    The node's XMLRPC URI is obtained from the master first.
"""

if __name__ == '__main__':
  parser = argparse.ArgumentParser( description='ROS Node XmlRpc client',
      usage='''rosxmlrpc --node NODENAME [OPTIONS] arguments ''')
  parser.add_argument('arguments', type = str, nargs = "*", help = 'method name followed by method arguments' )
  parser.add_argument('--node', "-n", type = str, default = "", help = 'Node name' )
  parser.add_argument('--caller_id', "-c", type = str, default = "anon", help = 'Caller ID' )
  parser.add_argument('--master_uri', "-m", type = str, default = os.environ.get( "ROS_MASTER_URI" ), help = 'ROS Master URI' )
  args = parser.parse_args()

  node_uri = get_node_uri( args.node, args.master_uri )
  if not node_uri:
    print( "Please provide node ( --node )" )
    sys.exit( 0 )

  method_name = args.arguments[0]
  method_args = args.arguments[1:]
  print( "Calling: %s( caller_id, %s )" % ( method_name, ", ".join( "'%s'" % a for a in method_args ) ) )

  rosxmlrpc = ROSXmlRpc( node_uri, args.caller_id )
  rosxmlrpc.call( method_name, method_args )
 

