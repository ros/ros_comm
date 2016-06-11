import collections
import os
import yaml

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

    def load_graph(self, graph_path=None):
        if graph_path is None:
            graph_path = self.graph_path
        # print("loading graph from %s" % graph_path)
        if not os.path.exists(graph_path):
            print("Requested graph file doesn't exist: [%s]" % graph_path)
            sys.exit(1)
        with open(graph_path, 'r') as f:
            self.graph = yaml.load(f)

    def save_graph(self, graph_path=None, sort_graph=True):
        if graph_path is None:
            graph_path = self.graph_path
        if sort_graph:
            self.graph = self.sortOD(self.graph)
        #print("saving graph to %s" % graph_path)
        #print('graph yaml:\n%s' % yaml.dump(graph))
        with open(graph_path, 'w') as f:
            f.write(yaml.dump(self.graph, default_flow_style=False))