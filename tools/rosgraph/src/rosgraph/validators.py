from __future__ import print_function

import logging
import os
# import names
# import time
# import socket
import traceback
# import ssl
import rosgraph_helper

import json
from copy import deepcopy
from sros_consts import ROLE_STRUCT, GraphModes

from cryptography import hazmat, x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.backends import default_backend

from apparmor.aare import re
from apparmor.common import convert_regexp


class PolicyInvalid(Exception):
    """Exception that is raised when a policy fails validation checks"""

    def __init__(self, message):
        self._message = message

    def __str__(self):
        return str(self._message)


class NameSpaceEngine(object):
    def __init__(self, node_name, _logger):
        self.node_name = node_name
        self._logger = _logger
        self._init_graph()

    def _init_graph(self):
        # ugly... need to figure out a graceful way to pass the graph
        self.graph_mode = GraphModes.enforce
        if 'SROS_POLICY_MODE' in os.environ:
            graph_mode = os.environ['SROS_POLICY_MODE']
            self.graph_mode = getattr(GraphModes, graph_mode, GraphModes.enforce)

        if self.node_name == 'master':
            self.graph = rosgraph_helper.GraphStructure()
            if 'SROS_POLICY_CONFIG' in os.environ:
                self.graph.graph_path = os.environ['SROS_POLICY_CONFIG']
                self.graph.load_graph()

    def _log_register(self, enable, flag, info):
        if enable:
            self._logger.info("{}REG [{}]:{} {} to graph:[{}]".format(flag, *info))

    def _is_profile_allowed(self, permitted, mode_name, role_name, action_name, caller_id):
        
        if permitted:
            return True
        else:
            # TODO fix crazy imports
            from security import caller_id_to_node_stem
            node_stem = caller_id_to_node_stem(caller_id)
            mask = ROLE_STRUCT[mode_name][role_name]['mask']
            info = (action_name, mask, 'self.node_stem', self.graph.graph_path)
            if self.graph_mode is GraphModes.enforce:
                allowed, audit = self.graph.is_allowed(node_stem, mode_name, action_name, mask)
                flag = '*' if allowed else '!'
                self._log_register(audit, flag, info)
                return allowed
            elif self.graph_mode is GraphModes.train:
                if self.graph.graph_path is not None:
                    allowed, audit = self.graph.is_allowed(node_stem, mode_name, action_name, mask)
                    if not allowed:
                        self.graph.add_allowed(node_stem, mode_name, action_name, mask)
                        self.graph.save_graph()
                    flag = '*' if allowed else '+'
                    self._log_register((audit or not allowed), flag, info)
                return True
            elif self.graph_mode is GraphModes.complain:
                if self.graph.graph is not None:
                    allowed, audit = self.graph.is_allowed(node_stem, mode_name, action_name, mask)
                    flag = '*' if allowed else '!'
                    self._log_register((audit or not allowed), flag, info)
                else:
                    self._log_register(True, '!', info)
                return True
            elif self.graph_mode is GraphModes.audit:
                if self.graph.graph is not None:
                    allowed, audit = self.graph.is_allowed(node_stem, mode_name, action_name, mask)
                    flag = '*' if allowed else '!'
                    self._log_register(True, flag, info)
                    return allowed
                else:
                    self._log_register(True, '', info)
                    return True
            else:
                return False

    def _is_action_allowed(self, permitted, mode=None, role=None, action=None):
        # info = (action, mask, self.node_stem, self.graph.graph_path)
        if self.graph_mode is GraphModes.enforce:
            # pass the judgment through
            return permitted
        elif self.graph_mode is GraphModes.train:
            # log any violation but let them slide
            return True
        elif self.graph_mode is GraphModes.complain:
            # pass the judgment through but log violations
            return permitted
        elif self.graph_mode is GraphModes.audit:
            # pass the judgment through but log everything
            return permitted
        else:
            return False

    def _is_policy_permited(self, policy, action):
        if 'policy_qualifiers' in policy['deny']:
            for qualifiers in policy['deny']['policy_qualifiers']:
                action_regex = re.compile(convert_regexp(qualifiers))
                if action_regex.search(action):
                    return False

        if 'policy_qualifiers' in policy['allow']:
            for qualifiers in policy['allow']['policy_qualifiers']:
                action_regex = re.compile(convert_regexp(qualifiers))
                if action_regex.search(action):
                    return True

        return False

    def _extract_policy(slef, cert, mode_name, role_name):
        policy = deepcopy(ROLE_STRUCT[mode_name][role_name])
        certificate_policies = cert.extensions.get_extension_for_class(x509.CertificatePolicies)
        for policy_information in certificate_policies.value:
            for mode in ['allow', 'deny']:
                if policy[mode]['OID'] == policy_information.policy_identifier.dotted_string:
                    policy[mode]['policy_qualifiers'] = policy_information.policy_qualifiers
        return policy

    def _extract_cert_from_sock(self, sock):
        cert_binary = sock.getpeercert(binary_form=True)
        cert = x509.load_der_x509_certificate(cert_binary, default_backend())
        return cert
    
    def _extract_cert_from_context(self, context):
        cert = x509.load_der_x509_certificate(context, default_backend())
        return cert

    def check_policy(self, sock, mode_name, role_name, action_name):
        cert = self._extract_cert_from_sock(sock)
        policy = self._extract_policy(cert, mode_name, role_name)
        permitted = self._is_policy_permited(policy, action_name)
        allowed = self._is_action_allowed(permitted)
        return policy, allowed

    def check_profile(self, context, mode_name, role_name, action_name, caller_id):
        cert = self._extract_cert_from_context(context)
        policy = self._extract_policy(cert, mode_name, role_name)
        permitted = self._is_policy_permited(policy, action_name)
        allowed = self._is_profile_allowed(permitted, mode_name, role_name, action_name, caller_id)
        return policy, allowed


class NameSpaceMasterAPI(object):
    
    def __init__(self, engine, _logger):
        self.engine = engine
        self._logger = _logger

    ###############################################################################
    # EXTERNAL API
    
    #TODO: How should external api calls be treated?
    # def shutdown(self, context, f):
    #     def check_permitted(instance, caller_id, msg):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, msg)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    # 
    #     return check_permitted
    # 
    # def getUri(self, context, f):
    #     def check_permitted(instance, caller_id):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    # 
    #     return check_permitted
    # 
    # def getPid(self, context, f):
    #     def check_permitted(instance, caller_id):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    # 
    #     return check_permitted

    ################################################################
    # PARAMETER SERVER ROUTINES

    #TODO add mutex locks in policy engine before enabling these high frequency calls 
    # def deleteParam(self, context, f):
    #     def check_permitted(instance, caller_id, key):
    #         policy, allowed = self.engine.check_profile(context, 'parameters', 'writer', key, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, key)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def setParam(self, context, f):
    #     def check_permitted(instance, caller_id, key, value):
    #         policy, allowed = self.engine.check_profile(context, 'parameters', 'writer', key, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, key, value)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def getParam(self, context, f):
    #     def check_permitted(instance, caller_id, key):
    #         policy, allowed = self.engine.check_profile(context, 'parameters', 'reader', key, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, key)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def searchParam(self, context, f):
    #     def check_permitted(instance, caller_id, key):
    #         policy, allowed = self.engine.check_profile(context, 'parameters', 'reader', key, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, key)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def subscribeParam(self, context, f):
    #     def check_permitted(instance, caller_id, caller_api, key):
    #         policy, allowed = self.engine.check_profile(context, 'parameters', 'reader', key, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, caller_api, key)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # #TODO: Should one only be able to un{subscribe,register} using one's own caller_id?
    # def unsubscribeParam(self, context, f):
    #     def check_permitted(instance, caller_id, caller_api, key):
    #         policy, allowed = self.engine.check_profile(context, 'parameters', 'reader', key, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, caller_api, key)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # #TODO: filter response through policy engine
    # # def hasParam(self, context, f):
    # #     def check_permitted(instance, caller_id, key):
    # #         policy, allowed = self.engine.check_profile(context, 'parameters', 'reader', key, caller_id)
    # #         if allowed:
    # #             return f(instance, caller_id, key)
    # #         else:
    # #             raise PolicyInvalid("ERROR: policy invalid for given action")
    # #     return check_permitted
    # # 
    # # def getParamNames(self, context, f):
    # #     def check_permitted(instance, caller_id):
    # #         policy, allowed = self.engine.check_profile(context, 'parameters', 'reader', key, caller_id)
    # #         if allowed:
    # #             return f(instance, caller_id)
    # #         else:
    # #             raise PolicyInvalid("ERROR: policy invalid for given action")
    # #     return check_permitted
    
    
    ##################################################################################
    # SERVICE PROVIDER
    
    def registerService(self, context, f):
        def check_permitted(instance, caller_id, service, service_api, caller_api):
            policy, allowed = self.engine.check_profile(context, 'services', 'server', service, caller_id)
            if allowed:
                return f(instance, caller_id, service, service_api, caller_api)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
    
        return check_permitted
    
    def lookupService(self, context, f):
        def check_permitted(instance, caller_id, service):
            policy, allowed = self.engine.check_profile(context, 'services', 'client', service, caller_id)
            if allowed:
                return f(instance, caller_id, service)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
    
        return check_permitted
    
    def unregisterService(self, context, f):
        def check_permitted(instance, caller_id, service, service_api):
            policy, allowed = self.engine.check_profile(context, 'services', 'server', service, caller_id)
            if allowed:
                return f(instance, caller_id, service, service_api)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
    
        return check_permitted


    ##################################################################################
    # PUBLISH/SUBSCRIBE

    def registerSubscriber(self, context, f):
        def check_permitted(instance, caller_id, topic, topic_type, caller_api):
            policy, allowed = self.engine.check_profile(context, 'topics', 'subscriber', topic, caller_id)
            if allowed:
                return f(instance, caller_id, topic, topic_type, caller_api)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
        return check_permitted

    def unregisterSubscriber(self, context, f):
        def check_permitted(instance, caller_id, topic, topic_type):
            policy, allowed = self.engine.check_profile(context, 'topics', 'subscriber', topic, caller_id)
            if allowed:
                return f(instance, caller_id, topic, topic_type)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
        return check_permitted
    
    def registerPublisher(self, context, f):
        def check_permitted(instance, caller_id, topic, topic_type, caller_api):
            policy, allowed = self.engine.check_profile(context, 'topics', 'publisher', topic, caller_id)
            if allowed:
                return f(instance, caller_id, topic, topic_type, caller_api)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
        return check_permitted
    
    def unregisterPublisher(self, context, f):
        def check_permitted(instance, caller_id, topic, topic_type):
            policy, allowed = self.engine.check_profile(context, 'topics', 'publisher', topic, caller_id)
            if allowed:
                return f(instance, caller_id, topic, topic_type)
            else:
                raise PolicyInvalid("ERROR: policy invalid for given action")
        return check_permitted


    ##################################################################################
    # GRAPH STATE APIS
    
    #TODO: How should graph state api calls be treated?
    # def lookupNode(self, context, f):
    #     def check_permitted(instance, caller_id, node_name):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, node_name)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def getPublishedTopics(self, context, f):
    #     def check_permitted(instance, caller_id, subgraph):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id, subgraph)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def getTopicTypes(self, context, f):
    #     def check_permitted(instance, caller_id):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    # 
    # def getSystemState(self, context, f):
    #     def check_permitted(instance, caller_id):
    #         policy, allowed = self.engine.check_profile(context, '?', '?', ?, caller_id)
    #         if allowed:
    #             return f(instance, caller_id)
    #         else:
    #             raise PolicyInvalid("ERROR: policy invalid for given action")
    #     return check_permitted
    

class NameSpaceSlaveAPI(object):
    def __init__(self, engine, _logger):
        self.engine = engine
        self._logger = _logger


class Transport(object):

    def connect_topic(self, sock, topic):
        return True

    def accept_topic(self, sock, topic):
        return True

    def connect_service(self, sock, service):
        return True

    def accept_service(self, sock, service):
        return True

class NameSpaceTransport(Transport):
    def __init__(self, engine, _logger):
        self.engine = engine
        self._logger = _logger
    
    def connect_topic(self, sock, topic):
        policy, allowed = self.engine.check_policy(sock, 'topics', 'publisher', topic)
        print('##################################################')
        print("policy:\n",json.dumps(policy, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('connect_topic', topic, allowed))
        print('##################################################')
        return allowed

    def accept_topic(self, sock, topic):
        policy, allowed = self.engine.check_policy(sock, 'topics', 'subscriber', topic)
        print('##################################################')
        print("policy:\n",json.dumps(policy, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('accept_topic', topic, allowed))
        print('##################################################')
        return allowed

    def connect_service(self, sock, service):
        policy, allowed = self.engine.check_policy(sock, 'services', 'server', service)
        print('##################################################')
        print("policy:\n",json.dumps(policy, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('connect_service', service, allowed))
        print('##################################################')
        return allowed

    def accept_service(self, sock, service):
        policy, allowed = self.engine.check_policy(sock, 'services', 'client', service)
        print('##################################################')
        print("policy:\n",json.dumps(policy, sort_keys=True, indent=4))
        print('Callback: {}\n action: {}\n allowed: {}'.format('accept_service', service, allowed))
        print('##################################################')
        return allowed