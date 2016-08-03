from __future__ import print_function

import logging
import os
# import names
# import time
# import socket
# import subprocess
import traceback
import ssl
# import shutil
import httplib
# import sys
from keyserver import get_keyserver_uri

from sros_consts import EXTENSION_MAPPING

try:
    import urllib.parse as urlparse #Python 3.x
except ImportError:
    import urlparse

try:
    import xmlrpc.client as xmlrpcclient #Python 3.x
except ImportError:
    import xmlrpclib as xmlrpcclient #Python 2.x

_logger = logging.getLogger('rosgraph.security')

#########################################################################

class Security(object):
    # TODO: add security logging stuff here
    def __init__(self, caller_id):
        if caller_id[0] is not '/':
            caller_id = '/' + caller_id
        self.node_id = caller_id
        self.node_stem = caller_id_to_node_stem(self.node_id)
        self.node_name = node_stem_to_node_name(self.node_stem)
        _logger.info("security init")

        import rosgraph.policy as policy
        policy.init(self.node_id, self.node_stem, self.node_name)
        self.policy = policy.get()
    def wrap_socket(self, sock):
        """
        Called whenever there is an opportunity to wrap a server socket.
        The default implementation doesn't do anything.
        """
        return sock
    def xmlrpc_protocol(self):
        return 'http'
    def get_context(self, sock):
        return None

#########################################################################

class NoSecurity(Security):

    def __init__(self, caller_id):
        super(NoSecurity, self).__init__(caller_id)
        _logger.info("  rospy.security.NoSecurity init")

    def xmlrpcapi(self, uri, context=None):
        uriValidate = urlparse.urlparse(uri)
        if not uriValidate[0] or not uriValidate[1]:
            return None
        return xmlrpcclient.ServerProxy(uri)

    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None):
        try:
            _logger.info('connecting to '+str(dest_addr)+' '+str(dest_port))
            sock.connect((dest_addr, dest_port))
        #except TransportInitError as tie:
        #    rospyerr("Unable to initiate TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))
        #    raise
        except Exception as e:
            _logger.warn("Unknown error initiating TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))
            # TODO: figure out how to bubble up and trigger the full close/release behavior in TCPROSTransport
            sock.close() # no reconnection as error is unknown
            raise TransportInitError(str(e)) #re-raise i/o error
        return sock

    def accept(self, server_sock, server_node_name):
        s = server_sock.accept()
        return (s[0], s[1])

#########################################################################

class XMLRPCTimeoutSafeTransport(xmlrpcclient.SafeTransport):
    def __init__(self, context=None, timeout=2.0):
        xmlrpcclient.SafeTransport.__init__(self, context=context)
        self.timeout = timeout
    def make_connection(self, host):
        chost, self._extra_headers, x509 = self.get_host_info(host)
        self._connection = host, httplib.HTTPSConnection(chost, None, timeout=self.timeout, context=self.context, **(x509 or {}))
        return self._connection[1]

#########################################################################
# global functions used by SSLSecurity and friends

def node_stem_to_node_name(node_stem):
    # get name from last element in stem
    node_name = node_stem.split('/')[-1]
    return node_name

def caller_id_to_node_stem(caller_id):
    # TODO: Check all practices that ROS uses to name nodes unique
    from rospy.names import canonicalize_name
    stem = canonicalize_name(caller_id)
    # check for anonymous node names. remove any long numeric suffix.
    # anonymous nodes will have numeric suffix tokens at the end (PID and time)
    tok = stem.split('_')    
    if len(tok) >= 3: # check if pid and epoch suffix posable
        if tok[-2].isdigit() and tok[-1].isdigit(): # check for pid and epoch
            if len(tok[-1]) == 13: # check for epoch 13 uses milliseconds
                stem = '_'.join(tok[0:-2])
    
    tok = stem.split('-')
    if len(tok) == 2: # check if for just hyphenated pid
        if tok[-1].isdigit(): # check if pid
            stem = tok[0]
    if len(tok) >= 3: # check if for just hyphenated pid
        if tok[-1].isdigit(): # check if pid
            stem = '-'.join(tok[0:-1])
    
    return stem

def open_private_output_file(fn):
    flags = os.O_WRONLY | os.O_CREAT
    return os.fdopen(os.open(fn, flags, 0o600), 'w')

class TLSSecurity(Security):

    def get_nodestore_paths(self):
        nodestore_paths = {}
        for key_cert, extension in EXTENSION_MAPPING.iteritems():
            file_name = self.node_name + extension
            nodestore_paths[key_cert] = os.path.join(self.nodestore_path, file_name)
        return nodestore_paths

    def nodestore_present(self):
        if not os.path.exists(self.nodestore_path):
            return False
        for key_cert, file_path in self.nodestore_paths.iteritems():
            if not os.path.isfile(file_path):
                return False
        return True

    def ca_present(self):
        return os.path.exists(self.capath)

    def init_nodestore(self):
        #TODO break this out into ksproxy like with rospy.msproxy?
        if not os.path.exists(self.nodestore_path):
            print("initializing node's keystore: %s" % self.nodestore_path)
            os.makedirs(self.nodestore_path)
            os.chmod(self.nodestore_path, 0o700)
        keyserver_proxy = self.xmlrpcapi(get_keyserver_uri(), context=self.get_keyserver_context())
        code, msg, value = keyserver_proxy.requestNodeStore(self.node_id, self.node_stem)
        for file_name, file_data in value.iteritems():
            file_path = os.path.join(self.nodestore_path, file_name)
            with open_private_output_file(file_path) as f:
                f.write(file_data)

    def init_ca(self):
        #TODO break this out into ksproxy like with rospy.msproxy?
        print("initializing node's capath: %s" % self.capath)
        os.makedirs(self.capath)
        os.chmod(self.capath, 0o700)

        keyserver_proxy = self.xmlrpcapi(get_keyserver_uri(), context=self.get_keyserver_context())
        code, msg, value = keyserver_proxy.getCA(self.node_id)
        for file_name, file_data in value.iteritems():
            file_path = os.path.join(self.capath, file_name)
            with open_private_output_file(file_path) as f:
                f.write(file_data)

    def init_context(self):
        if 'SROS_PASSWORD' in os.environ:
            password = os.environ['SROS_PASSWORD']
        else:
            password = None

        keyfile = self.nodestore_paths['key']
        certfile = self.nodestore_paths['cert']
        self.context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        self.context.verify_mode = ssl.CERT_REQUIRED
        self.context.load_verify_locations(capath=self.capath)
        self.context.load_cert_chain(certfile=certfile, keyfile=keyfile, password=password)

    def __init__(self, caller_id):
        super(TLSSecurity, self).__init__(caller_id)
        _logger.info("rospy.security.TLSSecurity init")

        self.keystore_path  = os.environ['SROS_KEYSTORE_PATH']
        self.capath   = os.path.join(self.keystore_path, 'capath')
        self.nodestore_path = os.path.join(self.keystore_path, 'nodes', self.node_stem.lstrip('/'))
        self.nodestore_paths = self.get_nodestore_paths()

        if not self.nodestore_present():
            self.init_nodestore()
        if not self.ca_present():
            self.init_ca()

        self.init_context()

        print('all startup certificates are present')

    def get_keyserver_context(self):
        context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        context.verify_mode = ssl.CERT_REQUIRED
        context.verify_mode = getattr(ssl, os.environ['SROS_KEYSERVER_VERIFY'])
        context.load_verify_locations(capath=self.capath)
        return context

    def wrap_socket(self, sock):
        """
        Called whenever there is an opportunity to wrap a server socket
        """
        return self.context.wrap_socket(sock, server_side=True)

    def xmlrpcapi(self, uri, context=None):
        # print("SSLSecurity.xmlrpcapi(%s, %s)" % (uri, node_stem or "[UNKNOWN]"))
        if context is None:
            context = self.context
        st = XMLRPCTimeoutSafeTransport(context=context, timeout=10.0)
        return xmlrpcclient.ServerProxy(uri, transport=st, context=context)

    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None):
        conn = self.context.wrap_socket(sock)
        try:
            conn.connect((dest_addr, dest_port))
        except Exception as e:
            print("  AHHH node %s SSLSecurity.connect() exception: %s" % (self.node_stem, e))
            raise
        return conn

    def accept(self, server_sock, server_node_name):
        # this is where peer-to-peer TCPROS connections try to be accepted
        # print("SSLSecurity.accept(server_node_name=%s)" % server_node_name)
        (client_sock, client_addr) = server_sock.accept()
        client_stream = None
        try:
            client_stream = self.context.wrap_socket(client_sock, server_side=True)
            # client_cert = client_stream.getpeercert()
        except Exception as e:
            print("SSLSecurity.accept() wrap_socket exception: %s" % e)
            raise
        return (client_stream, client_addr)
    
    def get_context(self, sock):
        cert = sock.getpeercert(binary_form=True)
        return cert

#########################################################################
_security = None
def init(caller_id):
    #TODO: sort out why we need this catch-all for master and roslaunch
    if caller_id.lstrip('/') not in ['master', 'roslaunch']:
        import rospy.names as names
        caller_id = names.get_caller_id()

    global _security
    if _security is None:
        _logger.info("choosing security model...")
        if 'SROS_SECURITY' in os.environ:
            if os.environ['SROS_SECURITY'] == 'ssl' or os.environ['SROS_SECURITY'] == 'ssl_setup':
                _security = TLSSecurity(caller_id)
            else:
                raise ValueError("illegal SROS_SECURITY value: [%s]" % os.environ['SROS_SECURITY'])
        else:
            _security = NoSecurity()

def get():
    if _security is None:
        import traceback
        traceback.print_stack()
        raise ValueError("woah there partner. security.init() wasn't called before security.get()")
    return _security
