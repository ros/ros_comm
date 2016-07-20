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

def parse_uri(uri):
    address_port = uri.split('://')[1]
    address, port = address_port.rstrip('/').split(':')
    return address, int(port)

def get_keyserver_uri():
    #TODO: Fix uri fetching
    master_uri = rosenv.get_master_uri()
    address, port = parse_uri(master_uri)
    keyserver_uri = 'https://%s:%d' % (address, port - 1)
    return keyserver_uri

class Keyserver(object):

    def __init__(self, keyserver_config, keystore_path, keyserver_mode):
        self.keyserver_config = keyserver_config
        self.keystore_path = keystore_path
        self.keyserver_mode = keyserver_mode

        self.uri = get_keyserver_uri()

        self.key_helper = key_helper.KeyHelper(self.keyserver_config, self.keystore_path)
        self.key_helper.init_ca()
        self.init_context()

        from rosgraph.xmlrpc import XmlRpcHandler
        class KeyserverHandler(XmlRpcHandler):
            """
            Base handler API for handlers used with XmlRpcNode. Public methods will be 
            exported as XML RPC methods.
            """

            def __init__(self, key_helper):
                self.key_helper = key_helper

            def requestNodeStore(self, node_stem):
                print('keyserver: requestNodeStore(%s)' % node_stem)
                resp = self.key_helper.get_nodestore(node_stem)
                return resp

            def getCA(self):
                print('keyserver: getCA()')
                resp = self.key_helper.get_ca()
                return resp

            def hello(self):
                return "I'm alive!"

        self.keyserver_handler = KeyserverHandler(self.key_helper)

    def init_context(self):
        capath = os.path.join(self.keystore_path, 'capath')
        certfile, keyfile = self.key_helper.init_keyserver()
        password = os.environ['SROS_KEYSERVER_PASSWORD']
        self.context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        self.context.verify_mode = getattr(ssl, self.keyserver_mode)
        self.context.load_verify_locations(capath=capath)
        self.context.load_cert_chain(certfile=certfile, keyfile=keyfile, password=password)

    def run(self):
        from rosgraph.xmlrpc import XmlRpcNode

        address, port = parse_uri(self.uri)

        keyserver_node = XmlRpcNode(
            port=port,
            rpc_handler=self.keyserver_handler,
            node_name='keyserver',
            context=self.context)

        try:
            keyserver_node.run()
        except KeyboardInterrupt:
            pass

def check_verify_mode(keyserver_mode):
    valid_modes = ['CERT_NONE','CERT_OPTIONAL','CERT_REQUIRED']
    if keyserver_mode in valid_modes:
        pass
    else:
        raise ValueError("\nFailed to fork keyserver, keyserver verify mode is invalid!\n" +
                         "User specified: {}".format(keyserver_mode)) 

def fork_xmlrpc_keyserver(keyserver_config, keystore_path, keyserver_mode):
    print("forking an unsecured XML-RPC server to bootstrap SSL key distribution...")

    check_verify_mode(keyserver_mode)

    keyserver = Keyserver(keyserver_config, keystore_path, keyserver_mode)

    from multiprocessing import Process
    p = Process(target=keyserver.run)
    p.start()
    # spin until the keyserver is responding to requests    
    from rosgraph.security import XMLRPCTimeoutSafeTransport
    st = XMLRPCTimeoutSafeTransport(context=keyserver.context, timeout=10.0)
    keyserver_proxy = xmlrpcclient.ServerProxy(keyserver.uri, transport=st, context=keyserver.context)

    print("sleeping until keyserver has generated the initial keyring...")
    while True:
        try:
            keyserver_proxy.hello()
            break
        except Exception as e:
            time.sleep(0.01)
    logging.getLogger('rosmaster.keyserver').info("Keyserver initialized: uri[%s]", keyserver.uri)
    print("horray, the keyserver is now open for business.")