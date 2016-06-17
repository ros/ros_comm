import collections
import os
import yaml

from apparmor.aare import re
from apparmor.common import convert_regexp

# Extend OrderedDict to support auto vivification
class ViviDict(collections.OrderedDict):
    def __missing__(self, key):
        value = self[key] = type(self)()
        return value

# Preserve sorted order of ViviDict when dumping to yaml
_mapping_tag = yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG
def dict_representer(dumper, data):
    return dumper.represent_dict(data.iteritems())
def dict_constructor(loader, node):
    return ViviDict(loader.construct_pairs(node))
yaml.add_representer(ViviDict, dict_representer)
yaml.add_constructor(_mapping_tag, dict_constructor)

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

    def interpret_graph(self):
        if 'nodes' in self.graph:
            for node_name, node_dict in self.graph['nodes'].iteritems():
                if 'regex' not in node_dict:
                    node_regex = re.compile(convert_regexp(node_name))
                    node_dict['regex'] = node_regex
                for topic_name, topic_dict in node_dict['topics'].iteritems():
                    if 'regex' not in topic_dict:
                        topic_regex = re.compile(convert_regexp(topic_name))
                        topic_dict['regex'] = topic_regex

    def un_interpret_graph(self, graph):
        if 'nodes' in graph:
            for node_name, node_dict in self.graph['nodes'].iteritems():
                if 'regex' in node_dict:
                    del node_dict['regex']
                for topic_name, topic_dict in node_dict['topics'].iteritems():
                    if 'regex' in topic_dict:
                        del topic_dict['regex']

    def load_graph(self, graph_path=None):
        if graph_path is None:
            graph_path = self.graph_path
        with open(graph_path, 'r') as f:
            self.graph = yaml.load(f)
        self.interpret_graph()

    def save_graph(self, graph_path=None, sort_graph=True):
        if graph_path is None:
            graph_path = self.graph_path
        if sort_graph:
            self.graph = self.sortOD(self.graph)
        #print("saving graph to %s" % graph_path)
        #print('graph yaml:\n%s' % yaml.dump(graph))
        self.graph['version'] = '0'

        graph_dump = self.graph.copy()
        self.un_interpret_graph(graph_dump)

        with open(graph_path, 'w') as f:
            f.write(yaml.dump(self.graph, default_flow_style=False))

    def check_for_mask(self, mask, masks, audit):
        index = masks.lower().find(mask)
        if index >= 0:
            return True, masks[index].isupper() or audit
        else:
            return False, audit

    def is_allowed(self, node_name, topic_name, mask):
        allow = False
        deny  = False
        audit = False
        allowed_nodes = self.graph['nodes']
        for node in allowed_nodes:
            allowed_node = allowed_nodes[node]
            if 'regex' in allowed_node:
                if allowed_node['regex'].search(node_name):
                    allowed_topics = allowed_node['topics']
                    for topic in allowed_topics:
                        allowed_topic = allowed_topics[topic]
                        if 'regex' in allowed_topic:
                            if allowed_topic['regex'].search(topic_name):
                                rules = allowed_topics[topic]
                                if 'deny' in rules:
                                    masks = rules['deny']
                                    deny, audit = self.check_for_mask(mask, masks, audit)
                                if 'allow' in rules:
                                    masks = rules['allow']
                                    allow, audit = self.check_for_mask(mask, masks, audit)

        allowed = allow and not deny
        return allowed, audit

    def add_allowed(self, node_name, topic_name, mask):
        allowed_node = self.graph['nodes'][node_name]
        allowed_topic_masks = allowed_node['topics'][topic_name]['allow']
        if type(allowed_topic_masks) is not str:
            allowed_topic_masks = ''
        allowed_topic_masks = ''.join(sorted(set((allowed_topic_masks + mask))))
        allowed_node['topics'][topic_name]['allow'] = allowed_topic_masks

        if 'regex' not in allowed_node:
            node_regex = re.compile(convert_regexp(node_name))
            allowed_node['regex'] = node_regex

        if 'regex' not in allowed_node['topics'][topic_name]:
            topic_regex = re.compile(convert_regexp(topic_name))
            allowed_node['topics'][topic_name]['regex'] = topic_regex