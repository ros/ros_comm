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

from cryptography import hazmat, x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.backends import default_backend

from apparmor.aare import re
from apparmor.common import convert_regexp

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
    def wrap_socket(self, sock):
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
    
    def allow_connect_subscriber(self, sock, dest_addr, dest_port, pub_uri, receive_cb, resolved_topic_name):
        return True
    
    def allow_topic_connection(self, sock, client_addr, header):
        return True
    
    def allow_call(self, sock, dest_addr, dest_port, service_uri):
        return True
    
    def allow_service_connection(self, sock, client_addr, header):
        return True

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

# def cert_cn(cert):
#     """
#     extract the commonName from this certificate
#     """
#     if cert is None:
#         return None
#     if not 'subject' in cert:
#         return None
#     for subject in cert['subject']:
#         key = subject[0][0]
#         if key == 'commonName':
#             return subject[0][1]
#     return None

##########################################################################

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
        if 'SROS_GRAPH_NAME' in os.environ:
            if '..' in os.environ['SROS_GRAPH_NAME'] or '/' in os.environ['SROS_GRAPH_NAME']:
                raise ValueError(
                    "graph name must be a simple string. saw [%s] instead" % os.environ['SROS_GRAPH_NAME'])
            self.graph.graph_path = os.path.join(os.path.expanduser('~'), '.ros', 'graphs',
                                                 os.environ['SROS_GRAPH_NAME'] + '.yaml')
            if os.path.exists(self.graph.graph_path):
                self.graph.load_graph()
            if 'SROS_GRAPH_MODE' in os.environ:
                graph_mode = os.environ['SROS_GRAPH_MODE']
                self.graph_mode = getattr(GraphModes, graph_mode, GraphModes.enforce)
        else:
            self.graph_mode = None

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
        if caller_id[0] is not '/':
            caller_id = '/' + caller_id
        self.node_id = caller_id
        self.node_stem = caller_id_to_node_stem(self.node_id)
        self.node_name = node_stem_to_node_name(self.node_stem)
        super(TLSSecurity, self).__init__()
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

        if self.node_name == 'master':
            self.init_graph()

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
            client_cert = client_stream.getpeercert()
            # print("SSLSecurity.accept(server_node_name=%s) getpeercert = %s" % (server_node_name, repr(client_cert)))
            # cn = cert_cn(client_cert)
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
        return (client_stream, client_addr)


    # def allow_xmlrpc_request(self, cert_text, cert_binary):
    #     cn = cert_cn(cert_text)
    #     if cn is None:
    #         return None
    #     # print("allow_xmlrpc_request() node=%s commonName=%s" % (self.node_stem, cn))
    #     # sanity-check to ensure that it ends in '.client'
    #     if not cn.endswith('.client'):
    #         return None
    #     try:
    #         # if we're master, make sure this cert exists in our keystore
    #         if self.node_stem == 'master':  # NOW I AM THE MASTER
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
    #             if cn_without_suffix == self.node_stem:
    #                 # print("    I'm talking to myself again.")
    #                 return cn_without_suffix  # it's always healthy to talk to yourself
    #             if cn_without_suffix in self.allowed_clients:
    #                 # print("    client %s is OK; it's in %s's allowed_clients list: %s" % (cn_without_suffix, self.node_stem, repr(self.allowed_clients)))
    #                 return cn_without_suffix
    #             else:
    #                 print(
    #                     "    client %s xmlrpc connection denied; it's not in %s's allowed list of clients: %s" % (
    #                     cn_without_suffix, self.node_stem, repr(self.allowed_clients)))
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
        node_stem = caller_id_to_node_stem(caller_id)
        info = (topic_name, mask, self.node_stem, self.graph.graph_path)
        if self.graph_mode is GraphModes.enforce:
            allowed, audit = self.graph.is_allowed(node_stem, topic_name, mask)
            flag = '*' if allowed else '!'
            self.log_register(audit, flag, info)
            return allowed
        elif self.graph_mode is GraphModes.train:
            if self.graph.graph_path is not None:
                allowed, audit = self.graph.is_allowed(node_stem, topic_name, mask)
                if not allowed:
                    self.graph.add_allowed(node_stem, topic_name, mask)
                    self.graph.save_graph()
                flag = '*' if allowed else '+'
                self.log_register((audit or not allowed), flag, info)
            return True
        elif self.graph_mode is GraphModes.complain:
            if self.graph.graph is not None:
                allowed, audit = self.graph.is_allowed(node_stem, topic_name, mask)
                flag = '*' if allowed else '!'
                self.log_register((audit or not allowed), flag, info)
            else:
                self.log_register(True, '!', info)
            return True
        elif self.graph_mode is GraphModes.audit:
            if self.graph.graph is not None:
                allowed, audit = self.graph.is_allowed(node_stem, topic_name, mask)
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
    
    def allow_policies(self, role, action): 

        if 'policy_qualifiers' in role['deny']:
            for qualifiers in role['deny']['policy_qualifiers']:
                action_regex = re.compile(convert_regexp(qualifiers))
                if action_regex.search(action):
                    return False

        if 'policy_qualifiers' in role['allow']:
            for qualifiers in role['allow']['policy_qualifiers']:
                action_regex = re.compile(convert_regexp(qualifiers))
                if action_regex.search(action):
                    return True

        return True

    def extract_policies(self, cert, role):
        certificate_policies = cert.extensions.get_extension_for_class(x509.CertificatePolicies)
        for policy_information in certificate_policies.value:
            for mode in ['allow', 'deny']:
                if role[mode]['OID'] == policy_information.policy_identifier.dotted_string:
                    role[mode]['policy_qualifiers'] = policy_information.policy_qualifiers

    def extract_cert(self, sock):
        cert_binary = sock.getpeercert(binary_form=True)
        cert = x509.load_der_x509_certificate(cert_binary, default_backend())
        return cert

    def allow_connect_subscriber(self, sock, dest_addr, dest_port, pub_uri, receive_cb, resolved_topic_name):
        cert = self.extract_cert(sock)
        role = deepcopy(ROLE_STRUCT['topics']['publisher'])
        self.extract_policies(cert, role)
        allowed = self.allow_policies(role, resolved_topic_name)
        print('##################################################')
        print('allow_connect_subscriber: ', allowed)
        print('##################################################')     
        return allowed

    def allow_topic_connection(self, sock, client_addr, header):
        cert = self.extract_cert(sock)
        role = deepcopy(ROLE_STRUCT['topics']['subscriber'])
        self.extract_policies(cert, role)
        allowed = self.allow_policies(role, header['topic'])
        print('##################################################')
        print('allow_topic_connection: ', allowed)
        print('##################################################')     
        return allowed

    def allow_call(self, sock, dest_addr, dest_port, service_uri):
        return True

    def allow_service_connection(self, sock, client_addr, header):
        return True

#########################################################################
_security = None
def init(caller_id):
    #print('security.init(%s)' % node_stem)
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
