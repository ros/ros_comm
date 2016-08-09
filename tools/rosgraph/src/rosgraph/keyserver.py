from __future__ import print_function

import logging
import os
# import names
import time
# import socket
# import subprocess
# import traceback
import ssl
import rosgraph.masterapi
from rosgraph import rosenv
# import shutil
# import httplib
# import sys
# import base64
# import rosgraph_helper
import key_helper
import sros_consts as sros_consts
#from rospy.exceptions import TransportInitError

try:
    import urllib.parse as urlparse #Python 3.x
except ImportError:
    import urlparse

try:
    import xmlrpc.client as xmlrpcclient #Python 3.x
except ImportError:
    import xmlrpclib as xmlrpcclient #Python 2.x

DEFAULT_KEYSERVER_PORT = 11310  # default port for keyservers's to bind to

class Keyserver(object):
    def __init__(self, keyserver_config, keystore_path, keyserver_mode, port=DEFAULT_KEYSERVER_PORT):
        self.keyserver_config = keyserver_config
        self.keystore_path = keystore_path
        self.keyserver_mode = keyserver_mode
        self.port = port

        self.key_helper = key_helper.KeyHelper(self.keyserver_config, self.keystore_path)
        self._init_context()

    def _init_context(self):
        capath = os.path.join(self.keystore_path, 'capath')
        certfile, keyfile = self.key_helper.init_keyserver()
        if 'SROS_KEYSERVER_PASSWORD' in os.environ:
            password = os.environ['SROS_KEYSERVER_PASSWORD']
        else:
            password = None
        self.context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        self.context.verify_mode = getattr(ssl, self.keyserver_mode)
        self.context.load_verify_locations(capath=capath)
        self.context.load_cert_chain(certfile=certfile, keyfile=keyfile, password=password)

    def start(self):
        """
        Start the SROS Keyserver.
        """
        self.handler = None
        self.keyserver_node = None
        self.uri = None

        from rosgraph.xmlrpc import XmlRpcNode
        from rosgraph.keyserver_api import KeyserverHandler
        handler = KeyserverHandler(self.key_helper)
        # keyserver_node = rosgraph.xmlrpc.XmlRpcNode(self.port, handler, node_name='keyserver')
        keyserver_node = XmlRpcNode(
            port=self.port,
            rpc_handler=handler,
            node_name='keyserver',
            context=self.context,
        )
        keyserver_node.start()

        # poll for initialization
        while not keyserver_node.uri:
            time.sleep(0.0001)

            # save fields
        self.handler = handler
        self.keyserver_node = keyserver_node
        self.uri = keyserver_node.uri

        logging.getLogger('sros.keyserver').info("Keyserver initialized: port[%s], uri[%s]", self.port, self.uri)

    def ok(self):
        if self.keyserver_node is not None:
            return self.keyserver_node.handler._ok()
        else:
            return False

    def stop(self):
        if self.keyserver_node is not None:
            self.keyserver_node.shutdown('Keserver.stop')
            self.keyserver_node = None


def check_verify_mode(keyserver_mode):
    if keyserver_mode in sros_consts.VerifyModes:
        pass
    else:
        raise ValueError("\nFailed to start keyserver, keyserver verify mode is invalid!\n" +
                         "User specified: {}".format(keyserver_mode))
    return keyserver_mode

def check_keyserver_alive(keyserver):
    # spin until the keyserver is responding to requests    
    from rosgraph.security import XMLRPCTimeoutSafeTransport
    st = XMLRPCTimeoutSafeTransport(context=keyserver.context, timeout=10.0)
    keyserver_proxy = xmlrpcclient.ServerProxy(keyserver.uri, transport=st, context=keyserver.context)
    print("sleeping until keyserver has generated the initial keyring...")
    while True:
        try:
            code, msg, value = keyserver_proxy.getUri('/keyserver')
            break
        except Exception as e:
            time.sleep(0.01)

    if value != keyserver.uri:
        raise ValueError("Keyserver URI does not match what was expexted:" + 
                         "getUri returned {} instead of {}".format(value, keyserver.uri))
    print("Horray, the keyserver is now open for business.")

def start_keyserver(keyserver_config, keystore_path, keyserver_mode, port):
    print("Starting an XML-RPC server to bootstrap SSL key distribution...")
    logger = logging.getLogger("sros.keyserver")
    logger.info("initialization complete, waiting for shutdown")

    try:
        logger.info("Starting SROS Keyserver Node")
        keyserver = Keyserver(keyserver_config, keystore_path, keyserver_mode, port)
        keyserver.start()
        check_keyserver_alive(keyserver)

        import time
        while keyserver.ok():
            time.sleep(.1)
    except KeyboardInterrupt:
        logger.info("keyboard interrupt, will exit")
    finally:
        logger.info("stopping keyserver...")
        keyserver.stop()