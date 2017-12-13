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
ROS Master authorization for Secure ROS
This class loads the rules from file and provides functions to 
check if clients (IP address) are authorized to make XMLRPC requests.
"""

from __future__ import print_function

import os
import yaml
import socket
import warnings
import logging
import rospkg
from itertools import chain
from urlparse import urlparse
from collections import OrderedDict


def getLogger( ):
    if getLogger.logger == None:
        ros_auth_log_level = logging.WARN
        if os.environ.has_key( "ROS_AUTH_LOG_LEVEL" ):
            ros_auth_log_level_str = os.environ.get( "ROS_AUTH_LOG_LEVEL" )
            if ros_auth_log_level_str.upper() == "DEBUG":
                ros_auth_log_level = logging.DEBUG
            elif ros_auth_log_level_str.upper() == "INFO":
                ros_auth_log_level = logging.INFO
            elif ros_auth_log_level_str.upper() == "WARN":
                ros_auth_log_level = logging.WARN
            elif ros_auth_log_level_str.upper() == "ERROR":
                ros_auth_log_level = logging.ERROR
            elif ros_auth_log_level_str.upper() == "FATAL":
                ros_auth_log_level = logging.FATAL
            else:
                print( "Environment variable ROS_AUTH_LOG_LEVEL set to '%s'. \n"
                        "It should be DEBUG, INFO, WARN, ERROR or FATAL. \n"
                        "Defaulting to %s" % ( ros_auth_log_level_str, logging.getLevelName( ros_auth_log_level ) ) )
        formatter = logging.Formatter('[auth][%(levelname)s] %(message)s')
        handler = logging.StreamHandler( )
        handler.setFormatter( formatter )
        handler.setLevel( ros_auth_log_level )
        getLogger.logger = logging.getLogger( "roslaunch.auth" )
        getLogger.logger.setLevel( logging.DEBUG )
        getLogger.logger.addHandler( handler )
    return getLogger.logger

getLogger.logger = None


def resolve_ip_address( ip_address ):
    """ Convert host or IP address to a list of IP addresses """
    try: 
        _, _, ip_address_list = socket.gethostbyname_ex( ip_address )
    except socket.gaierror as e:
        msg = "A %s exception occurred (%s)" % ( type(e).__name__, ", ".join( "%s" % a for a in e.args ) )
        getLogger().error( msg )
        msg = "Unable to resolve hostname: %s" % ( ip_address )
        getLogger().error( msg )
        raise ImportError( msg )
    except Exception as e:
        msg = "A %s exception occurred (%s)" % ( type(e).__name__, ", ".join( "%s" % a for a in e.args ) )
        getLogger().error( msg )
        raise e
    getLogger().debug( "%s resolved to %s" % ( ip_address, set( ip_address_list ) ) )
    return set( ip_address_list )



def resolve_ip_address_list( ip_address_list, aliases = dict() ):
    ip_address_set = set()
    for ip in ip_address_list:
        if ip in aliases.keys():
            ip_address_set.update( aliases[ip] )
        else:
            ip_address_set.update( resolve_ip_address( ip ) )
    return ip_address_set 
        


def uri_to_ip_address_list( uri ):
    """ Convert a URI to an IP address list
    """
    try:
        parsed_uri = urlparse( uri )
        _, _, ip_address_list = socket.gethostbyname_ex( parsed_uri.hostname )
    except Exception as e:
        getLogger().error( "Error: %s is not a valid URI (error=%s)" % ( uri, e.strerror ) )
        return []
    getLogger().debug( "uri_to_ip_address_list(%s) = %s" % ( parsed_uri.hostname, ip_address_list) )
    return ip_address_list 


def is_local_ip_address( ip_addr ):
    """ Check if the address is a local address, i.e. begins with a 127
    """
    return True if ip_addr.startswith( "127." ) or ip_addr.startswith( "169.254." ) else False


def is_local_uri( uri ):
    """ Check if the URI is a local address, i.e. begins with a 127
    """
    return any( [ is_local_ip_address( addr ) for addr in uri_to_ip_address_list( uri ) ] )


def is_uri_match( uri, ip_address ):
    """ Check if uri matches IP address
    """
    uri_ip_address_list = uri_to_ip_address_list( uri )
    if is_local_ip_address( ip_address ):
        if any( [ is_local_ip_address( addr ) for addr in uri_ip_address_list ] ):
            getLogger().debug( "is_uri_match( %s, %s ): OK (local IP address)" % ( uri, ip_address ) )
            return True
        else:
            getLogger().debug( "is_uri_match( %s, %s ): No (local IP address)" % ( uri, ip_address ) )
            return False
    else:
        if ip_address in uri_ip_address_list:
            getLogger().debug( "is_uri_match( %s, %s ): OK" % ( uri, ip_address ) )
            return True
        else:
            getLogger().debug( "is_uri_match( %s, %s ): No" % ( uri, ip_address ) )
            return False


class ROSMasterAuth():
    """
            publishers is a dictionary such that subscribers[topic] = list_of_subscriber_ip_addresses_for_that_topic
            subscribers is a dictionary such that publishers[topic] = list_of_publishers_ip_addresses_for_that_topic
            nodes is a dictionary such that nodes[node] = ip_address_for_that_node
            ip_addresses is a set of all IP addresses from which nodes, subscribers and publishers are allowed
            peer_publishers is a dictionary such that peer_publishers[ip_address] = set_of_publisher_ip_addresses_to_that_ip_address
    """
    reserved_parameters = ["/run_id", "/rosversion", "/rosdistro", "/tcp_keepalive", "/use_sim_time",
            "/enable_statistics", "/statistics_window_min_elements", "/statistics_window_max_elements", 
            "/statistics_window_min_size", "/statistics_window_max_size" ]
    reserved_topics = ["/rosout", "/rosout_agg"]
    reserved_nodes = ["/rosout"]

    def __init__( self, config_file = "", is_master = True ):
        self.noverify = False
        self.master = set()
        self.aliases = dict()
        self.subscribers = dict()
        self.publishers = dict()
        self.providers = dict()
        self.requesters = dict()
        self.methods = dict()
        self.nodes = dict()
        self.peer_publishers = dict()
        self.setters = dict()
        self.getters = dict()
        self.ip_addresses = set()
        self.logger = getLogger()

        """ Set ROS master IP address """
        if is_master: 
            if os.environ.has_key( "ROS_MASTER_URI" ):
                self.master = set( uri_to_ip_address_list( os.environ.get( "ROS_MASTER_URI" ) ) )
            else:
                raise RuntimeError( "ROS_MASTER_URI environment variable not set" )
        self.master.update( resolve_ip_address( "localhost" ) )
        self.logger.info( "Setting master from ROS_MASTER_URI: %s" % self.master )

        """ ROS authorization configuration file is stored in environment variable """
        if config_file == "":
            if os.environ.has_key( "ROS_AUTH_FILE" ):
                config_file = os.environ.get( "ROS_AUTH_FILE" )
        if config_file == "":
            self.noverify = True
            self.logger.warn( "Authorization configuration file not specified. Secure ROS is disabled!" )
        elif not os.path.exists( config_file ):
            raise RuntimeError( "Authorization configuration file %s does not exist" % config_file )
        else:
            self.logger.info( "Loading authorization from %s" % config_file )
            self.load( config_file )
        output_file = "%s/ros_auth_full.yaml" % rospkg.get_log_dir( env = os.environ )
        self.logger.info( "Writing full authorization to %s" % output_file )
        self.save( output_file )


    def enabled( self ):
        """ Is Secure ROS enabled?
            Return True if Secure ROS (i.e. authentication/verification) is enabled
        """
        return self.noverify == False 


    def resolve( self, ip_address_list ):
        """ Resolve ip_address_list where each ip_address may be an IP address, Host name or alias
        """
        return resolve_ip_address_list( ip_address_list, self.aliases )


    def load( self, config_file ):
        """ Load authorization file
        """
        doc = None
        try:
            with open( config_file, "r" ) as f:
                doc = yaml.load( f )
        except:
            raise ImportError( "Failed to read authorization file %s:" % config_file )
        if doc != None:
            if "aliases" in doc.keys():
                for key, val in doc["aliases"].items():
                    self.aliases[key] = resolve_ip_address_list( val )
                self.aliases = doc["aliases"]
            if "topics" in doc.keys():
                for k, v in doc["topics"].items():
                    if k in self.reserved_topics:
                        raise ImportError('Topic %s is a reserved topic!' % k )
                    if "publishers" in v.keys():
                        self.publishers[k] = self.resolve( v["publishers"] ) 
                    else:
                        raise ImportError('Topic %s does not have publishers!' % k )
                    if "subscribers" in v.keys():
                        self.subscribers[k] = self.resolve( v["subscribers"] ) | self.publishers[k]
                    else:
                        self.logger.warn('Topic %s does not have subscribers!' % k )
            else:
                raise ImportError( "Authorization file %s does not contain topics" % config_file )

            if "nodes" in doc.keys():
                for k, v in doc["nodes"].items():
                    if k in self.reserved_nodes:
                        raise ImportError('Topic %s is a reserved node!' % k )
                    self.nodes[k] = self.resolve( v )
            else:
                self.logger.warn( "Authorization file %s does not contain nodes" % config_file )

            if "services" in doc.keys():
                for k, v in doc["services"].items():
                    if "providers" in v.keys():
                        self.providers[k] = self.resolve( v["providers"] )
                    else:
                        raise ImportError('Service %s does not have providers!' % k )
                    if "requesters" in v.keys():
                        self.requesters[k] = self.resolve( v["requesters"] ) | self.providers[k]
                    else:
                        self.logger.warn('Service %s does not have requesters!' % k )
            else:
                self.logger.warn( "Authorization file %s does not contain services" % config_file )

            if "parameters" in doc.keys():
                for k, v in doc["parameters"].items():
                    if k in self.reserved_parameters:
                        raise ImportError('Parameter %s is a reserved topic!' % k )
                    if "setters" in v.keys():
                        self.setters[k] = self.resolve( v["setters"] )
                    else:
                        raise ImportError('Parameter %s does not have setters!' % k )
                    if "getters" in v.keys():
                        self.getters[k] = self.resolve( v["getters"] ) | self.setters[k]
                    else:
                        self.logger.warn( 'Parameter %s does not have getters!' % k )
            else:
                self.logger.warn( "Authorization file %s does not contain parameters" % config_file )
                pass
        else:
            raise ImportError( "Authorization file is empty" )

        """ set pub/sub for reserved topics """

        self.publishers["/rosout"] = set( ip for ips in chain( self.publishers.values(), self.subscribers.values() ) for ip in ips ) 
        self.subscribers["/rosout"] = self.master 

        self.publishers["/rosout_agg"] = self.master
        self.subscribers["/rosout_agg"] = self.master

        """ set ip_address for reserved nodes """

        self.nodes["/rosout"] = self.master

        """ compute peer publishers for each subscriber IP addresses """
        topics = set( self.publishers.keys() + self.subscribers.keys() )
        self.peer_publishers = { s: set() for subs in self.subscribers.values() for s in subs }
        for t in topics:
            for s in self.subscribers[t]:
                self.peer_publishers[s].update( set( self.publishers[t] ) )

        """ compute all authorized ip addresses """

        self.ip_addresses = set( ip for ip in chain.from_iterable( self.nodes.values() ) )
        self.ip_addresses.update( set( ip for ip in chain.from_iterable( self.publishers.values() ) ) )
        self.ip_addresses.update( set( ip for ip in chain.from_iterable( self.subscribers.values() ) ) )
        self.ip_addresses.update( set( ip for ip in chain.from_iterable( self.providers.values() ) ) )
        self.ip_addresses.update( set( ip for ip in chain.from_iterable( self.requesters.values() ) ) )
        self.ip_addresses.update( set( ip for ip in chain.from_iterable( self.setters.values() ) ) )
        self.ip_addresses.update( set( ip for ip in chain.from_iterable( self.getters.values() ) ) )

        """ set setters/getters for reserved parameters """
        for p in self.reserved_parameters:
            self.setters[p] = self.master
            self.getters[p] = self.ip_addresses
        

    def save( self, filename ):
        """ Write full authorization configuration to YAML file
        """
        data = OrderedDict()
        data.update( {"master": list( self.master ) } )
        data.update( {"aliases": { k: list( v ) for k, v in self.aliases.items() } } )
        data.update( {"nodes": { k: list( v ) for k, v in self.nodes.items() } } )
        data.update( {"publishers": { k: list( v ) for k, v in self.publishers.items() } } )
        data.update( {"subscribers": { k: list( v ) for k, v in self.subscribers.items() } } )
        data.update( {"peer_publishers": { k: list( v ) for k, v in self.peer_publishers.items() } } )
        data.update( {"setters": { k: list( v ) for k, v in self.setters.items() } } )
        data.update( {"getters": { k: list( v ) for k, v in self.getters.items() } } )
        data.update( {"providers": { k: list( v ) for k, v in self.providers.items() } } )
        data.update( {"requesters": { k: list( v ) for k, v in self.requesters.items() } } )
        data.update( {"ip_addresses": list( self.ip_addresses ) } )
        represent_dict_order = lambda self, data:  self.represent_mapping('tag:yaml.org,2002:map', data.items())
        yaml.add_representer(OrderedDict, represent_dict_order)    
        with open( filename, "w" ) as handle:
            yaml.dump( data, handle )


    def is_uri_match_or_noverify( self, uri, client_ip_address ):
        """ Check if URI matches client_ip_address only if noverify == False
        """
        if self.noverify == False:
            return is_uri_match( uri, client_ip_address )
        # return True if authorization check is disabled
        return True


    def check_key_ip_address( self, key, ip_address, auth_ip_addresses, query, fallback = False, prefix = False ):
        """ Check if ip_address is present in auth_ip_addresses[key]
            If prefix == True check if ip_address is present in auth_ip_addresses[key2] where key2 is a parent of key
            If there exists key2 which is a parent of 
            The parents of /a/bc/def/g are /a/bc/def, /a/bc, /a (not /a/b or /a/bc/de)
        """
        if self.noverify == True:
            self.logger.debug( "%s( %s, %s ): %s (noverify = True)" % ( query, key, ip_address, True ) )
            return True 
        try:
            if ip_address in self.ip_addresses:
                for key2, val2 in auth_ip_addresses.items():
                    if key == key2:
                        if ip_address in val2:
                            self.logger.debug( "%s( %s, %s ): %s" % ( query, key, ip_address, "OK" ) )
                            return True
                        else:
                            self.logger.debug( "%s( %s, %s ): %s (IP address not authorized)" % ( query, key, ip_address, "No" ) )
                            return False
                    if prefix and key.startswith( key2 ):
                        comp = key.split( "/" )
                        for i in range( 1, len( comp ) ):
                            if key2 == "/".join( c for c in comp[0:i+1] ):
                                if ip_address in val2:
                                    self.logger.debug( "%s( %s, %s ): %s (key prefix matches)" % ( query, key, ip_address, "OK" ) )
                                    return True
                                else:
                                    self.logger.warn( "%s( %s, %s ): %s (key prefix matches)" % ( query, key, ip_address, "No" ) )
                                    return False
                if fallback:
                    self.logger.debug( "%s( %s, %s ): %s (key not listed)" % ( query, key, ip_address, "OK" ) )
                else:
                    self.logger.warn( "%s( %s, %s ): %s (key not listed)" % ( query, key, ip_address, "No" ) )
                return fallback 
            self.logger.warn( "%s( %s, %s ): %s (unknown IP address)" % ( query, key, ip_address, "No" ) )
            return False
        except:
            self.logger.error( "Error querying %s( %s, %s )" % ( query, key, ip_address ) )
            return False


    def check_caller_id( self, caller_id, ip_address ):
        """ Check if caller_id is allowed from ip_address
                Allow caller_id if is not listed (fallback = True)
                Return True if allowed
        """
        return self.check_key_ip_address( caller_id, ip_address, self.nodes, "allow_node", fallback = True )


    def check_publisher( self, topic, ip_address ):
        """ Check if ip_address is allowed to publish to topic 
                Return True if allowed
        """
        return self.check_key_ip_address( topic, ip_address, self.publishers, "allow_publisher" )


    def check_subscriber( self, topic, ip_address ):
        """ Check if ip_address is allowed to publish to topic 
                Return True if allowed
        """
        return self.check_key_ip_address( topic, ip_address, self.subscribers, "allow_subscriber" )


    def check_method( self, method, ip_address ):
        """ Check if method is allowed from ip_address
                Allow method if it is not listed (fallback = True)
                Return True if allowed
        """
        return self.check_key_ip_address( method, ip_address, self.methods, "allow_method" )


    def check_peer_publisher_to_api( self, subscriber_uri, pub_ip_address ):
        """ Check if pub_ip_address is a publisher to sub_ip_address
                Return true if no verification 
        """
        for sub_ip_address in uri_to_ip_address_list( subscriber_uri ):
                if self.check_key_ip_address( sub_ip_address, pub_ip_address, self.peer_publishers, "allow_peer_publisher" ):
                        return True
        return False


    def check_provider( self, service, ip_address ):
        """ Check if ip_address is allowed to provide the service 
                Allow if service is not listed (fallback = True)
                Return True if allowed
        """
        return self.check_key_ip_address( service, ip_address, self.providers, "allow_provider", fallback = True )


    def check_requester( self, service, ip_address ):
        """ Check if ip_address is allowed to request the service 
                Allow if service is not listed (fallback = True)
                Return True if allowed
        """
        return self.check_key_ip_address( service, ip_address, self.requesters, "allow_requester", fallback = True )


    def check_param_setter( self, param, ip_address ):
        """ Check if param or prefix of param is allowed to be set from ip_address
                Allow if param is not listed (fallback = True)
                Return True if allowed
        """
        return self.check_key_ip_address( param, ip_address, self.setters, "allow_set_param", fallback = True, prefix = True )


    def check_param_getter( self, param, ip_address ):
        """ Return true if no verification 
        """
        return self.check_key_ip_address( param, ip_address, self.getters, "allow_get_param", fallback = True, prefix = True )


    def service_clients( self, service ):
        """ Return list of authorized clients for service 
            Return ["255.255.255.255"] if noverify = True
        """
        if self.noverify == True:
            self.logger.debug( "service_clients( %s ): noverify = True)" % (  service ) )
            return ["255.255.255.255",] 
        if service in self.requesters.keys():
            return list( self.requesters[service] )
        # if service is not listed, then all IP addresses in the config file are allowed
        return list( self.ip_addresses )

