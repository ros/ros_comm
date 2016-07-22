from __future__ import print_function

import logging
import os
# import names
# import time
# import socket
# import traceback
# import ssl
import rosgraph_helper

import json
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

_logger = logging.getLogger('rosgraph.policy')

#########################################################################

class Policy(object):
    # TODO: add security logging stuff here
    def __init__(self):
        _logger.info("policy init")

    def allow_registerPublisher(self, caller_id, topic, topic_type):
        print('Policy.allow_registerPublisher()')
        return True

    def allow_registerSubscriber(self, caller_id, topic, topic_type):
        print('Policy.allow_registerSubscriber()')
        return True

    def allow_xmlrpc_request(self, cert_text, cert_binary):
        return True

    def allow_connect_subscriber(self, sock, dest_addr, dest_port, pub_uri, receive_cb, resolved_topic_name):
        return True

    def allow_topic_connection(self, sock, client_addr, header):
        return True

    def allow_call(self, sock, dest_addr, dest_port, service_uri):
        return True

    def allow_service_connection(self, sock, client_addr, header):
        return True

#########################################################################

class NoPolicy(Policy):
    def __init__(self):
        super(NoPolicy, self).__init__()
        _logger.info("  rospy.policy.NoPolicy init")

##########################################################################

class NameSpacePolicy(Policy):

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

    def __init__(self, node_name):
        self.node_name = node_name
        super(NameSpacePolicy, self).__init__()
        _logger.info("rospy.policy.NameSpacePolicy init")

        if self.node_name == 'master':
            self.init_graph()

    def log_register(self, enable, flag, info):
        if enable:
            _logger.info("{}REG [{}]:{} {} to graph:[{}]".format(flag, *info))

    def allow_register(self, caller_id, topic_name, topic_type, mask):
        # TODO fix crazy imports
        from security import caller_id_to_node_stem
        
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

        return False

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
        print("role:\n",json.dumps(role, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('allow_connect_subscriber', resolved_topic_name, allowed))
        print('##################################################')
        return allowed

    def allow_topic_connection(self, sock, client_addr, header):
        cert = self.extract_cert(sock)
        role = deepcopy(ROLE_STRUCT['topics']['subscriber'])
        self.extract_policies(cert, role)
        allowed = self.allow_policies(role, header['topic'])
        print('##################################################')
        print("role:\n",json.dumps(role, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('allow_topic_connection', header['topic'], allowed))
        print('##################################################')
        return allowed

    def allow_call(self, sock, dest_addr, dest_port, resolved_name):
        cert = self.extract_cert(sock)
        role = deepcopy(ROLE_STRUCT['services']['server'])
        self.extract_policies(cert, role)
        allowed = self.allow_policies(role, resolved_name)
        print('##################################################')
        print("role:\n",json.dumps(role, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('allow_call', resolved_name, allowed))
        print('##################################################')
        return allowed

    def allow_service_connection(self, sock, client_addr, header):
        cert = self.extract_cert(sock)
        role = deepcopy(ROLE_STRUCT['services']['client'])
        self.extract_policies(cert, role)
        allowed = self.allow_policies(role, header['service'])
        print('##################################################')
        print("role:\n",json.dumps(role, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('allow_service_connection', header['service'], allowed))
        print('##################################################')
        return allowed


#########################################################################
# _policy = None
# 
# 
# def init(caller_id):
#     # print('security.init(%s)' % node_stem)
#     global _policy
#     if _policy is None:
#         _logger.info("choosing policy model...")
#         if 'SROS_SECURITY' in os.environ:
#             if os.environ['SROS_SECURITY'] == 'ssl' or os.environ['SROS_SECURITY'] == 'ssl_setup':
#                 _policy = NameSpacePolicy(caller_id)
#             else:
#                 raise ValueError("illegal SROS_SECURITY value: [%s]" % os.environ['SROS_SECURITY'])
#         else:
#             _policy = NoPolicy()
# 
# 
# def get():
#     if _policy is None:
#         import traceback
#         traceback.print_stack()
#         raise ValueError("woah there partner. policy.init() wasn't called before policy.get()")
#     return _policy
