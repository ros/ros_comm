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
from rosgraph import rosenv
import shutil
import httplib
import sys
import base64
import rosgraph_helper
import key_helper
#from rospy.exceptions import TransportInitError

try:
    import urllib.parse as urlparse #Python 3.x
except ImportError:
    import urlparse

try:
    import xmlrpc.client as xmlrpcclient #Python 3.x
except ImportError:
    import xmlrpclib as xmlrpcclient #Python 2.x



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




_key_helper = None


def parse_uri(uri):
    address_port = uri.split('://')[1]
    address, port = address_port.rstrip('/').split(':')
    return address, int(port)


def get_keyserver_uri():
    master_uri = rosenv.get_master_uri()
    address, port = parse_uri(master_uri)
    # keyserver_uri = 'http://%s:%d' % (address, port - 1)
    #TODO: Fix uri fetching
    keyserver_uri = 'http://%s:%d' % ('127.0.0.1', port - 1)
    print("##########################################\nkeyserver_uri: {}\n######################################".format(keyserver_uri))
    return keyserver_uri


def keyserver_requestNodeStore(node_name):
    node_name = node_name_to_cert_stem(node_name) # sanitize and de-anonymize
    print('keyserver: requestNodeStore(%s)' % node_name)
    resp = _key_helper.get_nodestore(node_name)
    return resp


def keyserver_getCA():
    print('keyserver: getCA()')
    resp = _key_helper.get_ca()
    return resp


def keyserver_hello():
    return "I'm alive!"


def keyserver_main(config_path, keys_dir):
    global _key_helper
    from SimpleXMLRPCServer import SimpleXMLRPCServer
    from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler

    _key_helper = key_helper.KeyHelper(config_path, keys_dir)
    _key_helper.init_ca()

    server = SimpleXMLRPCServer(parse_uri(get_keyserver_uri()), SimpleXMLRPCRequestHandler, False)
    server.register_function(keyserver_requestNodeStore, 'requestNodeStore')
    server.register_function(keyserver_getCA, 'getCA')
    server.register_function(keyserver_hello, 'hello') # it answers just to say it's alive
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


def fork_xmlrpc_keyserver(config_path, keys_dir):
    print("forking an unsecured XML-RPC server to bootstrap SSL key distribution...")
    from multiprocessing import Process
    p = Process(target=keyserver_main,args=(config_path, keys_dir))
    p.start()
    # spin until the keyserver is responding to requests
    keyserver_proxy = xmlrpcclient.ServerProxy(get_keyserver_uri())
    print("sleeping until keyserver has generated the initial keyring...")
    while True:
        try:
            keyserver_proxy.hello()
            break
        except Exception as e:
            time.sleep(0.01)
    print("horray, the keyserver is now open for business.")