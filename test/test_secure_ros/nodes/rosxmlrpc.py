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
import xml


class ROSXmlRpc( ):
  def __init__( self, master_uri, caller_id = "anon" ):
    self.master_uri = master_uri
    self.caller_id = caller_id
    self.client = xmlrpclib.ServerProxy( self.master_uri )

  def call( self, method_name, method_args ):
    if not hasattr( self.client, method_name ):
      print( 'Unrecognized ROS XMLRPC method: %s' % method_name )
      return
    method = getattr( self.client, method_name )
    ret, msg, val = method( self.caller_id, *tuple( method_args ) )
    print( "Returned: %d" % ret )
    print( "Message: %s" % msg )
    if type( val ) == list:
        print( "Value:\n%s" % "\n".join( "- %s" % v for v in val ) )
    else:
        print( "Value: %s" % val )
    return ret, msg, val 
  

if __name__ == '__main__':
  parser = argparse.ArgumentParser( description='ROS Master XmlRpc client',
      usage='''rosxmlrpc <command> [OPTIONS] [<args>]"

The ROS XMLRPC commands are:

  getServiceClients       Get list of authorized subscriber clients
  getPublishedTopics      Get list of published topics
  getSystemState          Get list of published topics

''')
  parser.add_argument('arguments', type = str, nargs = "*", help = 'Method name plus arguments' )
  parser.add_argument('--caller_id', "-c", type = str, default = "anon", help = 'Caller ID' )
  parser.add_argument('--master_uri', "-m", type = str, default = os.environ.get( "ROS_MASTER_URI" ), help = 'ROS Master URI' )
  args = parser.parse_args()

  method_name = args.arguments[0]
  method_args = args.arguments[1:]
  print( "Calling: %s( caller_id, %s )" % ( method_name, ", ".join( "'%s'" % a for a in method_args ) ) )

  rosxmlrpc = ROSXmlRpc( args.master_uri, args.caller_id )
  rosxmlrpc.call( method_name, method_args )
 
