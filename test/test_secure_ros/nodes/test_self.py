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


"""
Read in an authorization file and test Secure ROS implementation by probing master.
The assumption is that all nodes are running.
"""

from rosmaster.authorization import ROSMasterAuth
import xmlrpclib
import logging
import sys
import os
import argparse

def getLogger( ):
  """ Create test logger
  """
  if getLogger.logger == None:
    formatter = logging.Formatter('[test][%(levelname)s] %(message)s')
    handler = logging.StreamHandler( )
    handler.setFormatter( formatter )
    handler.setLevel( logging.INFO )
    getLogger.logger = logging.getLogger( "test" )
    getLogger.logger.setLevel( logging.DEBUG )
    getLogger.logger.addHandler( handler )
  return getLogger.logger

getLogger.logger = None


def get_authorized( ip_addr, auth_file ):
  """ Create list of authorized topics, services, etc 
      This is done outside the rosmaster.authorization module as an independent test
      (It uses rosmaster.authorization to parse the YAML file)
  """
  print( "Checking authorization for %s" % ip_addr )
  auth_list = {}
  auth = ROSMasterAuth( auth_file, is_master = False )
  auth_file2 = "auth.yaml" 
  getLogger().info( "Saving authorization configuration to %s" % auth_file2 )
  auth.save( auth_file2 )
  auth_list["published_topics"] = set( t for (t,ip_set) in auth.publishers.items() if ip_addr in ip_set )
  auth_list["subscribed_topics"] = set( t for (t,ip_set) in auth.subscribers.items() if ip_addr in ip_set )
  auth_list["state"] = { }
  auth_list["state"]["published_topics"] = set( t for (t,ip_set) in auth.publishers.items() if ip_addr in ip_set )
  auth_list["state"]["subscribed_topics"] = set( t for (t,ip_set) in auth.subscribers.items() if ip_addr in ip_set )
  auth_list["state"]["published_topics"] -= {"/rosout_agg"}

  return auth_list



class Tester():
  def __init__( self, master_uri, ip_addr, auth_file ):
    self.logger = getLogger()
    self.logger.info( "ROS Master: %s" % master_uri )
    self.logger.info( "IP addr: %s" % ip_addr )
    self.logger.info( "Authorization configuration file: %s" % auth_file )
    self.results = {}
    self.master_uri = master_uri
    self.ip_addr = ip_addr
    self.auth_list = get_authorized( ip_addr, auth_file )
    self.client = xmlrpclib.ServerProxy( self.master_uri )
    self.caller_id = "tester" 


  def call( self, method_name, args ):
    method = getattr( self.client, method_name )
    self.logger.debug( "%s( %s )" % ( method_name, ", ".join( "'%s'" % a for a in args ) ) )
    retval = method( *args )
    return retval


  def test_method( self, method_name, args = tuple() ):
    """ Call XMLRPC method_name with value
    """
    args = (self.caller_id,) + tuple(args)
    ret, msg, val = self.call( method_name, args )
    if ret != 1:
      self.logger.warn( "%s: Call Failed (%s, %s)" % ( method_name, ret, msg ) )
    else:
      self.logger.debug( "%s: Call OK" % method_name )
    return val 


  def check_test_equal( self, method, actual_value, reference_value, value_name ):
    """ Check if actual_value == reference_value 
    """
    name = "%s_%s_equal" % ( method, value_name )
    self.results[name] = 0
    if actual_value == reference_value:
      self.results[name] = 1
      self.logger.debug( "%s: OK" % method )
    else:
      self.logger.warn( "%s: Failed. %s: %s (reference: %s)" % ( method, 
        value_name, actual_value, reference_value ) )
    return self.results[name]


  def print_results( self ):
    total = len( self.results )
    success = sum( v for k, v in self.results.items() )
    self.logger.info( "===" )
    self.logger.info( "Successful: %d/ %d" % ( success, total ) )

  def test_topics( self ):
    """ """
    self.logger.info( "-- Testing topics --" )

    #method = "lookupNode"
    #val = self.test_method( method, ("",) )
    #topics = set( v[0] for v in val )
    #self.check_test_equal( method, topics, self.auth_list.topics["topics"], )

    """ """
    method = "getPublishedTopics"
    val = self.test_method( method, ("",) )
    topics = set( v[0] for v in val )
    self.check_test_equal( method, topics, self.auth_list["subscribed_topics"], "topics" )

    """ """
    method = "getTopicTypes"
    val = self.test_method( method )
    topics = set( v[0] for v in val )
    self.check_test_equal( method, topics, self.auth_list["subscribed_topics"], "topics" )

    """ """
    success = True
    method = "getSystemState"
    publishers, subscribers, services = self.test_method( method )
    sub_topics = set( v[0] for v in subscribers )
    self.check_test_equal( method, sub_topics, self.auth_list["state"]["published_topics"], "published_topics" )
    pub_topics = set( v[0] for v in publishers )
    self.check_test_equal( method, pub_topics, self.auth_list["state"]["subscribed_topics"], "subscribed_topics" )
    """ """



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument( "--auth_file", "-a", type = str, default = os.environ.get( "ROS_AUTH_FILE" ), help="Authorization file" )
  parser.add_argument( "--master_uri", "-M", type = str, default = os.environ.get( "ROS_MASTER_URI" ), help = 'Master URI' )
  parser.add_argument( "--ip_addr", "-I", type = str, default = os.environ.get( "ROS_IP" ), help = 'IP address' )
  args = parser.parse_args()
  if args.ip_addr == None:
    print( "Unable to obtain IP address. Please use the -I option to specify IP address of this machine." )
    exit()
  if args.auth_file == None:
    print( "Unable to obtain authorization file. Please use the -a option to specify configuration file." )
    exit()
  elif not os.path.exists( args.auth_file ):
    print( "Authorization file (%s) does not exist. Please use the -a option to specify configuration file." % args.auth_file )
    exit()

  auth_file = os.path.realpath( args.auth_file )
  tester = Tester( args.master_uri, args.ip_addr, auth_file )
  tester.test_topics()
  tester.print_results()

