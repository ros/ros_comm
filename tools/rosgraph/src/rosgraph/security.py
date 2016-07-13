from __future__ import print_function

import logging
import os
import names
import time
import socket
import subprocess
import traceback
import ssl
import rosgraph.masterapi
import shutil
import httplib
import sys
import base64
import rosgraph_helper
import key_helper
from keyserver import get_keyserver_uri
#from rospy.exceptions import TransportInitError

from copy import deepcopy
from sros_consts import ROLE_STRUCT, EXTENSION_MAPPING

class GraphModes:
    audit = 'audit'
    complain = 'complain'
    enforce = 'enforce'
    train = 'train'

    class __metaclass__(type):
        def __contains__(self, item):
            return hasattr(self, item)

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
    def __init__(self):
        _logger.info("security init")
    def wrap_socket(self, sock, node_name):
        """
        Called whenever there is an opportunity to wrap a server socket.
        The default implementation doesn't do anything.
        """
        return sock
    def xmlrpc_protocol(self):
        return 'http'
    def allow_registerPublisher(self, caller_id, topic, topic_type):
        print('Security.allow_registerPublisher()')
        return True
    def allow_registerSubscriber(self, caller_id, topic, topic_type):
        print('Security.allow_registerSubscriber()')
        return True
    def allow_xmlrpc_request(self, cert_text, cert_binary):
        return True
    def allowClients(self, caller_id, clients):
        return 1, "", 0

#########################################################################

class NoSecurity(Security):

    def __init__(self):
        super(NoSecurity, self).__init__()
        _logger.info("  rospy.security.NoSecurity init")

    def xmlrpcapi(self, uri, node_name = None):
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
        return (s[0], s[1], 'unknown')

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
def node_name_to_cert_stem(node_name):
    node_name = caller_id_to_node_name(node_name)
    stem = node_name
    if (stem[0] == '/'):
        stem = stem[1:]
    if '..' in node_name:
        raise ValueError('woah there. node_name [%s] has a double-dot. OH NOES.' % node_name)
    return stem.replace('/', '__')

def caller_id_to_node_name(caller_id):
    stem = caller_id
    # check for anonymous node names. remove any long numeric suffix.
    # anonymous nodes will have numeric suffix tokens at the end (PID and time)
    tok = stem.split('_')
    if len(tok) >= 3: # check if pid and epoch suffix posable
        if tok[-2].isdigit() and tok[-1].isdigit(): # check for pid and epoch
            if len(tok[-1]) == 13: # check for epoch 13 uses milliseconds
                stem = '_'.join(tok[0:-2])
    return stem

def open_private_output_file(fn):
    flags = os.O_WRONLY | os.O_CREAT
    return os.fdopen(os.open(fn, flags, 0o600), 'w')

def cert_cn(cert):
    """
    extract the commonName from this certificate
    """
    if cert is None:
        return None
    if not 'subject' in cert:
        return None
    for subject in cert['subject']:
        key = subject[0][0]
        if key == 'commonName':
            return subject[0][1]
    return None

##########################################################################

class TLSSecurity(Security):

    def get_nodestore_paths(self):

        nodestore_paths = deepcopy(ROLE_STRUCT)
        for role_name, role_struct in nodestore_paths.iteritems():
            for mode_name, mode_struct in role_struct.iteritems():
                file_mode = role_name + '.' + mode_name
                file_name = self.node_name + '.' + file_mode
                role_struct[mode_name] = {}
                for key_cert, extension in EXTENSION_MAPPING.iteritems():
                    file_path = os.path.join(self.nodestore_path, role_name, file_name + extension)
                    role_struct[mode_name][key_cert] = file_path
        return nodestore_paths

    def nodestore_present(self):
        if not os.path.exists(self.nodestore_path):
            return False
        for role_name, role_struct in self.nodestore_paths.iteritems():
            for mode_name, mode_struct in role_struct.iteritems():
                for key_cert, file_path in mode_struct.iteritems():
                    if not os.path.isfile(file_path):
                        return False
        return True

    def ca_present(self):
        return os.path.exists(self.ca_path)

    def init_nodestore(self):
        if not os.path.exists(self.nodestore_path):
            print("initializing node's keystore: %s" % self.nodestore_path)
            os.makedirs(self.nodestore_path)
            os.chmod(self.nodestore_path, 0o700)
        keyserver_proxy = xmlrpcclient.ServerProxy(get_keyserver_uri())
        response = keyserver_proxy.requestNodeStore(self.node_name)
        for type_name, type_data in response.iteritems():
            type_path = os.path.join(self.nodestore_path, type_name)
            if not os.path.exists(type_path):
                os.makedirs(type_path)
            for file_name, file_data in type_data.iteritems():
                file_path = os.path.join(type_path, file_name)
                # print("Saving {} to: {}".format(file_name, file_path))
                if os.path.isfile(file_path):
                    continue
                with open_private_output_file(file_path) as f:
                    f.write(file_data)

    def init_ca(self):
        print("initializing node's ca_path: %s" % self.ca_path)
        os.makedirs(self.ca_path)
        os.chmod(self.ca_path, 0o700)

        keyserver_proxy = xmlrpcclient.ServerProxy(get_keyserver_uri())
        response = keyserver_proxy.getCA()
        for file_name, file_data in response.iteritems():
            file_path = os.path.join(self.ca_path, file_name)
            # print("Saving {} to: {}".format(file_name, file_path))
            if os.path.isfile(file_path):
                continue
            with open_private_output_file(file_path) as f:
                f.write(file_data)

    def init_graph(self):
        # some extra initialization steps for graph saving/loading
        self.graphs_path = os.path.join(os.path.expanduser('~'), '.ros', 'graphs')
        if not os.path.exists(self.graphs_path):
            print("initializing empty graph store: %s" % self.graphs_path)
            os.makedirs(self.graphs_path)
            os.chmod(self.graphs_path, 0o700)

        # ugly... need to figure out a graceful way to pass the graph
        self.graph_mode = GraphModes.enforce
        self.graph = rosgraph_helper.GraphStructure()
        if 'ROS_GRAPH_NAME' in os.environ:
            if '..' in os.environ['ROS_GRAPH_NAME'] or '/' in os.environ['ROS_GRAPH_NAME']:
                raise ValueError(
                    "graph name must be a simple string. saw [%s] instead" % os.environ['ROS_GRAPH_NAME'])
            self.graph.graph_path = os.path.join(os.path.expanduser('~'), '.ros', 'graphs',
                                                 os.environ['ROS_GRAPH_NAME'] + '.yaml')
            if os.path.exists(self.graph.graph_path):
                self.graph.load_graph()
            if 'ROS_GRAPH_MODE' in os.environ:
                graph_mode = os.environ['ROS_GRAPH_MODE']
                self.graph_mode = getattr(GraphModes, graph_mode, GraphModes.enforce)
        else:
            self.graph_mode = None

    def init_contexts(self):
        self.nodestore_contexts = deepcopy(self.nodestore_paths)
        for role_name, role_struct in self.nodestore_contexts.iteritems():
            for mode_name, mode_struct in role_struct.iteritems():
                keyfile = mode_struct['key']
                certfile = mode_struct['cert']

                context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
                context.verify_mode = ssl.CERT_REQUIRED
                context.load_verify_locations(capath=self.ca_path)
                context.load_cert_chain(certfile=certfile, keyfile=keyfile)
                role_struct[mode_name] = context

    def __init__(self, node_name):
        self.node_name = node_name_to_cert_stem(node_name)
        super(TLSSecurity, self).__init__()
        _logger.info("rospy.security.TLSSecurity init")

        self.keystore_path  = os.environ['ROS_KEYSTORE_PATH']
        self.ca_path   = os.path.join(self.keystore_path, 'ca_path')
        self.nodestore_path = os.path.join(self.keystore_path, 'nodes', self.node_name)
        self.nodestore_paths = self.get_nodestore_paths()

        self.client_contexts_= {}
        self.server_context_ = None

        if not self.nodestore_present():
            self.init_nodestore()
        if not self.ca_present():
            self.init_ca()

        self.init_contexts()

        print('all startup certificates are present')

        if node_name == 'master':
            self.init_graph()

    def get_server_context(self, role, mode):
        if role is None:
            role = 'topics'
        if mode is None:
            mode = 'publisher'
        return self.nodestore_contexts[role][mode]

    def get_client_context(self, role, mode):
        if role is None:
            role = 'topics'
        if mode is None:
            mode = 'subscriber'
        return self.nodestore_contexts[role][mode]

    def wrap_socket(self, sock, node_name, role=None, mode=None):
        """
        Called whenever there is an opportunity to wrap a server socket
        """
        # print("SSLSecurity.wrap_socket() called for node %s" % node_name)
        context = self.get_server_context(role, mode)
        return context.wrap_socket(sock, server_side=True)  # connstream

    def xmlrpcapi(self, uri, role=None, mode=None):
        # print("SSLSecurity.xmlrpcapi(%s, %s)" % (uri, node_name or "[UNKNOWN]"))
        context = self.get_client_context(role, mode)
        st = XMLRPCTimeoutSafeTransport(context=context, timeout=10.0)
        return xmlrpcclient.ServerProxy(uri, transport=st, context=context)

    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None, role=None, mode=None):
        # print('SSLSecurity.connect() to node at %s:%d which is endpoint %s' % (dest_addr, dest_port, endpoint_id))
        context = self.get_client_context(role, mode)
        conn = context.wrap_socket(sock)
        try:
            conn.connect((dest_addr, dest_port))
        except Exception as e:
            print("  AHHH node %s SSLSecurity.connect() exception: %s" % (self.node_name, e))
            raise
        return conn

    def accept(self, server_sock, server_node_name, role=None, mode=None):
        # this is where peer-to-peer TCPROS connections try to be accepted
        # print("SSLSecurity.accept(server_node_name=%s)" % server_node_name)
        context = self.get_server_context(role, mode)
        (client_sock, client_addr) = server_sock.accept()
        client_stream = None
        try:
            client_stream = context.wrap_socket(client_sock, server_side=True)
            client_cert = client_stream.getpeercert()
            # print("SSLSecurity.accept(server_node_name=%s) getpeercert = %s" % (server_node_name, repr(client_cert)))
            cn = cert_cn(client_cert)
            # if cn is None:
            #     raise Exception("unknown certificate format")
            # if not cn.endswith('.client'):
            #     raise Exception("unexpected commonName format: %s" % cn)
            # # get rid of the '.client' suffix
            # cn = '.'.join(cn.split('.')[0:-1])
            # if not cn in self.allowed_clients:
            #     raise Exception("unknown client [%s] is trying to connect!" % cn)
        except Exception as e:
            print("SSLSecurity.accept() wrap_socket exception: %s" % e)
            raise
        print("%s is allowing inbound connection from %s" % (self.node_name, cn))
        return (client_stream, client_addr, cn)

    # only ever called by master
    def allowClients(self, caller_id, clients):
        # print("node=%s allowClients(%s)" % (self.node_name, repr(clients)))
        # for client in clients:
        #     sanitized_name = node_name_to_cert_stem(client)
        #     # print("  allowed_clients.add(%s)" % sanitized_name)
        #     self.allowed_clients.add(sanitized_name)
        # # todo: save certificate in our keystore?
        # # todo: allow only for specific topics, not always all-access?
        return 1, "", 0

    # def allow_xmlrpc_request(self, cert_text, cert_binary):
    #     cn = cert_cn(cert_text)
    #     if cn is None:
    #         return None
    #     # print("allow_xmlrpc_request() node=%s commonName=%s" % (self.node_name, cn))
    #     # sanity-check to ensure that it ends in '.client'
    #     if not cn.endswith('.client'):
    #         return None
    #     try:
    #         # if we're master, make sure this cert exists in our keystore
    #         if self.node_name == 'master':  # NOW I AM THE MASTER
    #             path = os.path.join(self.keystore, cn + '.cert')
    #             if '..' in path:
    #                 print('HEY WHAT ARE YOU TRYING TO DO WITH THOSE DOTS')
    #                 return None
    #             # print('looking for client cert in [%s]' % path)
    #             if not os.path.isfile(path):
    #                 print("WOAH THERE PARTNER. I DON'T KNOW [%s]" % cn)
    #                 return None
    #             with open(path, 'r') as client_cert_file:
    #                 file_contents = client_cert_file.read()
    #                 # remove header and footer lines
    #                 file_contents = ''.join(file_contents.split('\n')[1:-2])
    #             # print('file contents: %s' % file_contents)
    #             client_cert = base64.b64encode(cert_binary)
    #             # print('cert transmitted: %s' % client_cert)
    #             if file_contents != client_cert:
    #                 print('WOAH. cert mismatch!')
    #                 return None
    #             else:
    #                 # print('    cert matches OK')
    #                 return cn[:-7]
    #         else:  # we're not the master. life is more complicated.
    #             # see if it's the master
    #             if cn == 'master.client':
    #                 # we have this one in our keystore; we can verify it
    #                 cert_path = os.path.join(self.keystore, 'master.client.cert')
    #                 with open(cert_path, 'r') as cert_file:
    #                     saved_cert = cert_file.read()
    #                     # remove header and footer lines
    #                     saved_cert = ''.join(saved_cert.split('\n')[1:-2])
    #                 presented_cert = base64.b64encode(cert_binary)
    #                 if saved_cert != presented_cert:
    #                     print("WOAH. master client certificate mismatch!")
    #                     return None
    #                 else:
    #                     # print("   master client cert matches OK")
    #                     return 'master'
    #             # see if we are talking to ourselves
    #             cn_without_suffix = '.'.join(cn.split('.')[0:-1])
    #             if cn_without_suffix == self.node_name:
    #                 # print("    I'm talking to myself again.")
    #                 return cn_without_suffix  # it's always healthy to talk to yourself
    #             if cn_without_suffix in self.allowed_clients:
    #                 # print("    client %s is OK; it's in %s's allowed_clients list: %s" % (cn_without_suffix, self.node_name, repr(self.allowed_clients)))
    #                 return cn_without_suffix
    #             else:
    #                 print(
    #                     "    client %s xmlrpc connection denied; it's not in %s's allowed list of clients: %s" % (
    #                     cn_without_suffix, self.node_name, repr(self.allowed_clients)))
    #                 return None
    #
    #     except Exception as e:
    #         print('oh noes: %s' % e)
    #         return None
    #     print("WOAH WOAH WOAH how did i get here?")
    #     return 'ahhhhhhhhhhhh'

    def log_register(self, enable, flag, info):
        if enable:
            _logger.info("{}REG [{}]:{} {} to graph:[{}]".format(flag, *info))

    def allow_register(self, caller_id, topic_name, topic_type, mask):
        node_name = caller_id_to_node_name(caller_id)
        info = (topic_name, mask, node_name, self.graph.graph_path)
        if self.graph_mode is GraphModes.enforce:
            allowed, audit = self.graph.is_allowed(node_name, topic_name, mask)
            flag = '*' if allowed else '!'
            self.log_register(audit, flag, info)
            return allowed
        elif self.graph_mode is GraphModes.train:
            if self.graph.graph_path is not None:
                allowed, audit = self.graph.is_allowed(node_name, topic_name, mask)
                if not allowed:
                    self.graph.add_allowed(node_name, topic_name, mask)
                    self.graph.save_graph()
                flag = '*' if allowed else '+'
                self.log_register((audit or not allowed), flag, info)
            return True
        elif self.graph_mode is GraphModes.complain:
            if self.graph.graph is not None:
                allowed, audit = self.graph.is_allowed(node_name, topic_name, mask)
                flag = '*' if allowed else '!'
                self.log_register((audit or not allowed), flag, info)
            else:
                self.log_register(True, '!', info)
            return True
        elif self.graph_mode is GraphModes.audit:
            if self.graph.graph is not None:
                allowed, audit = self.graph.is_allowed(node_name, topic_name, mask)
                flag = '*' if allowed else '!'
                self.log_register(True, flag, info)
                return allowed
            else:
                self.log_register(True, '', info)
                return True
        else:
            return False

    def allow_registerPublisher(self, caller_id, topic, topic_type):
        if self.graph_mode is None:
            return True
        else:
            return self.allow_register(caller_id, topic, topic_type, 'w')

    def allow_registerSubscriber(self, caller_id, topic, topic_type):
        if self.graph_mode is None:
            return True
        else:
            return self.allow_register(caller_id, topic, topic_type, 'r')

    def xmlrpc_protocol(self):
        return 'https'

#########################################################################
_security = None
def init(node_name):
    #print('security.init(%s)' % node_name)
    global _security
    if _security is None:
        _logger.info("choosing security model...")
        if 'ROS_SECURITY' in os.environ:
            if os.environ['ROS_SECURITY'] == 'ssl' or os.environ['ROS_SECURITY'] == 'ssl_setup':
                _security = TLSSecurity(node_name)
            else:
                raise ValueError("illegal ROS_SECURITY value: [%s]" % os.environ['ROS_SECURITY'])
        else:
            _security = NoSecurity()

def get():
    if _security is None:
        import traceback
        traceback.print_stack()
        raise ValueError("woah there partner. security.init() wasn't called before security.get()")
    return _security
