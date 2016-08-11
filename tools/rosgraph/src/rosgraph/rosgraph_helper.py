import collections
import os
import yaml
import copy
import pickle

from apparmor.aare import re
from apparmor.common import convert_regexp

# Extend OrderedDict to support auto vivification
class ViviDict(collections.OrderedDict):
    #TODO: Fix deepcopy
    def __missing__(self, key):
        value = self[key] = type(self)()
        return value

# Preserve sorted order of ViviDict when dumping to yaml
_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG
def dict_representer(dumper, data):
    return dumper.represent_dict(data.iteritems())
def ViviDict_constructor(loader, node):
    return ViviDict(loader.construct_pairs(node))
def dict_constructor(loader, node):
    return ViviDict(loader.construct_pairs(node))

###############################################################
# GRAPH STUFF
###############################################################

class GraphStructure:
    def __init__(self):
        self.graph_path = None
        self.graph = ViviDict()

    def sortOD(self, od):
        if od is None:
            return od
        res = ViviDict()
        for k, v in sorted(od.items()):
            if isinstance(v, dict):
                res[k] = self.sortOD(v)
            else:
                res[k] = v
        return res

    def compile_graph(self):
        if 'nodes' in self.graph:
            for node_name, node_dict in self.graph['nodes'].iteritems():
                for role_name, role_dict in node_dict.iteritems():
                    for action_name, action_dict in role_dict.iteritems():
                        if 'regex' not in action_dict:
                            action_regex = re.compile(convert_regexp(action_name))
                            action_dict['regex'] = action_regex
                if 'regex' not in node_dict:
                    node_regex = re.compile(convert_regexp(node_name))
                    node_dict['regex'] = node_regex

    def uncompile_graph(self, graph):
        if 'nodes' in graph:
            for node_name, node_dict in graph['nodes'].iteritems():
                if 'regex' in node_dict:
                    node_dict.pop('regex')
                for role_name, role_dict in node_dict.iteritems():
                    for action_name, action_dict in role_dict.iteritems():
                        if 'regex' in action_dict:
                            action_dict.pop('regex')

    def load_graph(self, graph_path=None):
        if graph_path is None:
            graph_path = self.graph_path
        with open(graph_path, 'r') as f:
            yaml.add_representer(ViviDict, dict_representer)
            yaml.add_constructor(_mapping_tag, ViviDict_constructor)
            self.graph = yaml.load(f)
            yaml.add_representer(dict, dict_representer)
            yaml.add_constructor(_mapping_tag, dict_constructor)
        self.compile_graph()

    def save_graph(self, graph_path=None, sort_graph=True, save_regex=False):
        if graph_path is None:
            graph_path = self.graph_path
        dump_graph = pickle.loads(pickle.dumps(self.graph))
        with open(graph_path, 'w') as f:
            f.write(self.graph_dump(dump_graph, sort_graph, save_regex))

    def graph_dump(self, dump_graph, sort_graph=True, save_regex=False):
        if sort_graph:
            dump_graph = self.sortOD(dump_graph)

        if not save_regex:
            self.uncompile_graph(dump_graph)

        self.graph['version'] = '0'

        yaml.add_representer(ViviDict, dict_representer)
        yaml.add_constructor(_mapping_tag, ViviDict_constructor)
        dump = yaml.dump(dump_graph, default_flow_style=False)
        yaml.add_representer(dict, dict_representer)
        yaml.add_constructor(_mapping_tag, dict_constructor)

        return dump

    def check_for_mask(self, mask, masks, audit):
        index = masks.lower().find(mask)
        if index >= 0:
            return True, masks[index].isupper() or audit
        else:
            return False, audit

    def is_allowed(self, node_name, role_name, action_name, mask):
        allow = False
        deny  = False
        audit = False
        allowed_nodes = self.graph['nodes']
        for node in allowed_nodes:
            allowed_node = allowed_nodes[node]
            if 'regex' in allowed_node:
                if allowed_node['regex'].search(node_name):
                    allowed_actions = allowed_node[role_name]
                    for action in allowed_actions:
                        allowed_action = allowed_actions[action]
                        if 'regex' in allowed_action:
                            if allowed_action['regex'].search(action_name):
                                rules = allowed_actions[action]
                                if 'deny' in rules:
                                    masks = rules['deny']
                                    deny, audit = self.check_for_mask(mask, masks, audit)
                                if 'allow' in rules:
                                    masks = rules['allow']
                                    allow, audit = self.check_for_mask(mask, masks, audit)

        allowed = allow and not deny
        return allowed, audit

    def add_allowed(self, node_name, role_name, action_name, mask):
        allowed_node = self.graph['nodes'][node_name]
        allowed_action_masks = allowed_node[role_name][action_name]['allow']
        if type(allowed_action_masks) is not str:
            allowed_action_masks = ''
        allowed_action_masks = ''.join(sorted(set(allowed_action_masks + mask)))
        allowed_node[role_name][action_name]['allow'] = allowed_action_masks

        if 'regex' not in allowed_node:
            node_regex = re.compile(convert_regexp(node_name))
            allowed_node['regex'] = node_regex

        if 'regex' not in allowed_node[role_name][action_name]:
            action_regex = re.compile(convert_regexp(action_name))
            allowed_node[role_name][action_name]['regex'] = action_regex

    def filter_nodes(self, namespace_filter):
        if 'nodes' not in self.graph:
            return None
        else:
           return self.filter_namespaces(self.graph['nodes'], namespace_filter)

    def filter_namespaces(self, graph, namespace_filter):
        if graph is None:
            return None
        else:
            filtered_graph = pickle.loads(pickle.dumps(graph))
            for namespace, policy in filtered_graph.iteritems():
                if not policy['regex'].search(namespace_filter):
                    filtered_graph.pop(namespace)
            return filtered_graph

    def filter_policies(self, graph, policy_filter):
        if graph is None:
            return None
        else:
           filtered_graph = pickle.loads(pickle.dumps(graph))
           for namespace, policy in filtered_graph.iteritems():
               for policy_type, policy_type_data in policy.iteritems():
                   if policy_type != policy_filter:
                       policy.pop(policy_type)
           return filtered_graph

    def filter_modes(self, graph, mode_filter):
        if graph is None:
            return None
        else:
           filtered_graph = pickle.loads(pickle.dumps(graph))
           for namespace, policy in filtered_graph.iteritems():
               for policy_type, policy_type_data in policy.iteritems():
                   for policy_namespace, policy_namespace_data in policy_type_data.iteritems():
                       if mode_filter not in policy_namespace_data:
                           policy_type_data.pop(policy_namespace)
           return filtered_graph

    def filter_masks(self, graph, mask_filter):
        if graph is None:
            return None
        else:
           filtered_graph = pickle.loads(pickle.dumps(graph))
           for namespace, policy in filtered_graph.iteritems():
               for policy_type, policy_type_data in policy.iteritems():
                   for policy_namespace, policy_namespace_data in policy_type_data.iteritems():
                       for policy_mode, policy_mode_data in policy_namespace_data.iteritems():
                           try:
                               if mask_filter not in policy_mode_data.lower():
                                   policy_type_data.pop(policy_namespace)
                           except:
                               pass
           return filtered_graph

    def list_policy_namespaces(self, graph):
        if graph is None:
            return None
        else:
            policy_namespaces_list = []
            for namespace, policy in graph.iteritems():
                for policy_type, policy_type_data in policy.iteritems():
                    for policy_namespace, policy_namespace_data in policy_type_data.iteritems():
                        policy_namespaces_list.append(policy_namespace)
            return policy_namespaces_list