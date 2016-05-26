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
#from rospy.exceptions import TransportInitError

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


#########################################################################

class NoSecurity(Security):
    def __init__(self):
        super(NoSecurity, self).__init__()
        _logger.info("  rospy.security.NoSecurity init")
    def xmlrpcapi(self, uri, node_name):
        uriValidate = urlparse.urlparse(uri)
        if not uriValidate[0] or not uriValidate[1]:
            return None
        return xmlrpcclient.ServerProxy(uri)
    def connect(self, sock, dest_addr, dest_port, endpoint_id, pub_node_name, timeout=None):
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

#########################################################################

class SSHSecurity(Security):
    def __init__(self):
        super(SSHSecurity, self).__init__()
        _logger.info("  rospy.security.SSHSecurity init")
        self.ssh_tunnels = { }

    def xmlrpcapi(self, uri, node_name):
        uriValidate = urlparse.urlparse(uri)
        if not uriValidate[0] or not uriValidate[1]:
            return None
        requested_uri = uri
        _logger.info("rospy.security.xmlrpcapi(%s) host = %s" % (uri, uriValidate.hostname))

        # see if we should connect directly to this URI,
        # or instead fork an SSH tunnel and go through that
        dest_hostname = uriValidate.hostname
        dest_port = uriValidate.port
        # TODO: if we are connecting locally, don't worry about it
        # for now, let's tunnel everything to easily test its functionality.
        # if uriValidate.hostname != 'localhost' and uriValidate.hostname != '127.0.0.1':
        if True:
            # see if we already have a tunnel to this URI
            if uri in self.ssh_tunnels:
                uri = self.ssh_tunnels[uri]['local_uri']
            else:
                #print(" need to create a tunnel!")
                # first, ask the OS for a free port to listen on
                port_probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                port_probe.bind(('', 0))
                local_port = port_probe.getsockname()[1]
                port_probe.close() # the OS won't re-allocate this port anytime soon
                ssh_incantation = ["ssh", "-nNT", "-L", "%d:localhost:%d" % (local_port, dest_port), str(dest_hostname)]
                #print("spawning ssh tunnel with: %s" % " ".join(ssh_incantation))
                try:
                    self.ssh_tunnels[uri] = {}
                    self.ssh_tunnels[uri]['local_uri'] = "http://localhost:%d" % local_port
                    self.ssh_tunnels[uri]['local_port'] = local_port
                    self.ssh_tunnels[uri]['process'] = subprocess.Popen(ssh_incantation, shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    time.sleep(1) # HAXX. do a retry-loop
                    uri = self.ssh_tunnels[uri]['local_uri']
                except Exception as e:
                    print("OH NOES couldn't create tunnel: %s" % str(e))
                    raise
        _logger.info("remapped xmlrpcapi request: %s => %s" % (requested_uri, uri))
        return xmlrpcclient.ServerProxy(uri)

    def connect(self, sock, dest_addr, dest_port, endpoint_id, pub_node_name, timeout=None):
        # TODO: if we are connecting locally, don't worry about it
        # for now, let's tunnel everything to easily test its functionality.
        # if dest_addr != 'localhost' and dest_addr != '127.0.0.1':
        if True:
            port_probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            port_probe.bind(('', 0))
            local_port = port_probe.getsockname()[1]
            port_probe.close() # the OS won't re-allocate this port anytime soon
            ssh_incantation = ["ssh", "-nNT", "-L", "%d:localhost:%d" % (local_port, dest_port), str(dest_addr)]
            #print("spawning ssh tunnel with: %s" % " ".join(ssh_incantation))
            try:
                subprocess.Popen(ssh_incantation, shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                time.sleep(1) # HAXX. do a retry-loop
                sock.connect(('127.0.0.1', local_port))
            except Exception as e:
                print("OH NOES couldn't connect through tunnel: %s" % str(e))
                raise

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
    # TODO: convert anonymous names into something consistent, so we
    # don't end up generating arbitrary numbers of them
    stem = node_name
    if (stem[0] == '/'):
        stem = stem[1:]
    if '..' in node_name:
        raise ValueError('woah there. node_name [%s] has a double-dot. OH NOES.' % node_name)
    return stem.replace('/', '__')

#########################################################################
class SSLCertificateCreator(object):
    def __init__(self):
        print("SSLCertificateCreator()")
        self.kpath = os.path.join(os.path.expanduser('~'), '.ros', 'keys', 'master')
        if not os.path.exists(self.kpath):
            print("initializing empty roscore keystore: %s" % self.kpath)
            os.makedirs(self.kpath)
        self.openssl_conf_path = os.path.join(self.kpath, 'openssl.conf')
        self.root_ca_path = os.path.join(self.kpath, 'root.ca')
        self.root_cert_path = os.path.join(self.kpath, 'root.cert')
        self.create_roscore_certs_if_needed()

    def run_and_print_abnormal_output(self, cmd, status_text=None):
        #subprocess.check_call(cmd.split(' '))
        popen = subprocess.Popen(cmd.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdoutdata, stderrdata = popen.communicate()
        if status_text is not None:
            sys.stdout.write("%s..." % status_text)
            sys.stdout.flush()
        #print("running command: %s" % cmd)
        if popen.returncode != 0:
            print("error running command: [%s]" % cmd)
            print("    stdout:\n%s\n" % stdoutdata)
            print("    stderr:\n%s\n" % stderrdata)
            raise Exception("ahhh")
        if status_text is not None:
            print("done")

    def create_cert(self, node_name, suffix):
        _logger.info("SSLCertificateCreator.create_cert(%s, %s)" % (node_name, suffix))
        if not names.is_legal_base_name(node_name):
            raise ValueError("SSLCertificateCreator.create_cert() illegal node name: %s" % node_name)
        cert_name = node_name + '.' + suffix
        csr_path = os.path.join(self.kpath, '%s.csr' % cert_name)
        key_path = os.path.join(self.kpath, '%s.key' % cert_name)
        if not os.path.isfile(csr_path):
            openssl_incantation = r'openssl req -batch -subj /C=US/ST=CA/O=ros/CN={0} -newkey rsa:4096 -sha256 -nodes -out {1} -keyout {2}'.format(cert_name, csr_path, key_path)
            self.run_and_print_abnormal_output(openssl_incantation)

        cert_path = os.path.join(self.kpath, '%s.cert' % cert_name)
        if not os.path.isfile(cert_path):
            openssl_incantation = "openssl ca -batch -config {0} -notext -in {1} -out {2}".format(self.openssl_conf_path, csr_path, cert_path)
            self.run_and_print_abnormal_output(openssl_incantation, "creating %s certificates for node [%s]" % (suffix, node_name))

    def create_certs_if_needed(self, node_name):
        fn = [ node_name + '.server.cert',
               node_name + '.server.key',
               node_name + '.client.cert',
               node_name + '.client.key' ]
        all_found = True
        for f in fn:
            if not os.path.isfile(os.path.join(self.kpath, f)):
                all_found = False
        if all_found:
            return # nothing to do
        # otherwise, we need to generate certificates for this node
        self.create_cert(node_name, 'server')
        self.create_cert(node_name, 'client')

    def create_roscore_certs_if_needed(self):
        self.root_key_path = os.path.join(self.kpath, 'root.key')
        if not os.path.isfile(self.root_cert_path) or \
           not os.path.isfile(self.root_key_path):
            openssl_incantation = "openssl req -batch -newkey rsa:4096 -subj /C=US/ST=CA/O=ros -days 3650 -x509 -sha256 -nodes -out %s -keyout %s" % (self.root_cert_path, self.root_key_path)
            self.run_and_print_abnormal_output(openssl_incantation, "creating self-signed root certificate authority")

        if not os.path.isfile(self.root_ca_path):
            with open(self.root_ca_path, 'w') as root_ca_file:
                with open(self.root_key_path, 'r') as root_key_file:
                    root_ca_file.write(root_key_file.read())
                with open(self.root_cert_path, 'r') as root_cert_file:
                    root_ca_file.write(root_cert_file.read())

        root_db_path = os.path.join(self.kpath, 'certindex')
        if not os.path.isfile(root_db_path):
            open(root_db_path, 'w').close() # creates an empty file

        root_serial_path = os.path.join(self.kpath, 'serial')
        if not os.path.isfile(root_serial_path):
            with open(root_serial_path, 'w') as f:
                f.write('000a')

        # TODO: need to set extendedKeyUsage to clientAuth for client certs
        if not os.path.isfile(self.openssl_conf_path):
            with open(self.openssl_conf_path, 'w') as f:
                f.write('''\
[ ca ]
default_ca = ros_ca

[ crl_ext ]
authorityKeyIdentifier=keyid:always

[ ros_ca ]
new_certs_dir = /tmp
unique_subject = no
certificate = {0}
database = {1}
private_key = {2}
serial = {3}
default_days = 3650
default_md = sha512
policy = ros_ca_policy
x509_extensions = ros_ca_extensions

[ ros_ca_policy ]
commonName = supplied
stateOrProvinceName = optional
countryName = optional
emailAddress = optional
organizationName = optional
organizationalUnitName = optional

[ ros_ca_extensions ]
basicConstraints = CA:false
subjectKeyIdentifier = hash
authorityKeyIdentifier = keyid:always
keyUsage = digitalSignature,keyEncipherment,keyAgreement
extendedKeyUsage = serverAuth,clientAuth
#subjectAltName = @alt_names
#
#[ alt_names ]
#DNS.1 = localhost
'''.format(self.root_cert_path, root_db_path, self.root_key_path, root_serial_path))
        self.create_cert('master','server')
        self.create_cert('master','client')
        self.create_cert('roslaunch','server')
        self.create_cert('roslaunch','client')

##########################################################################

class SSLSecurity(Security):
    def __init__(self, node_name):
        self.node_name = node_name_to_cert_stem(node_name)
        super(SSLSecurity, self).__init__()
        _logger.info("rospy.security.SSLSecurity init")
        self.kpath = os.path.join(os.path.expanduser('~'), '.ros', 'keys', self.node_name)

        self.master_server_cert_path = self.keystore_path('master', '.server.cert')
        self.client_cert_verify_mode = ssl.CERT_REQUIRED 
        self.server_cert_verify_mode = ssl.CERT_REQUIRED

        self.server_context_ = None
        self.client_contexts_ = {}

        if not self.all_certs_present():
            if not os.path.exists(self.kpath):
                print("initializing empty node keystore: %s" % self.kpath)
                os.makedirs(self.kpath)
            keyserver_proxy = xmlrpcclient.ServerProxy(keyserver_addr())
            print("calling keyserver_proxy.getCertificates()")
            response = keyserver_proxy.getCertificates(self.node_name)
            print("response: %s" % response)
            for fn, contents in response.iteritems():
                path = os.path.join(self.kpath, fn)
                print("  writing [%s]" % path)
                if os.path.isfile(path):
                    continue
                with open(path, 'w') as f:
                    f.write(contents)
    
    def all_certs_present(self):
        if not os.path.exists(self.kpath):
            return False
        fn = [ 'root.cert', 'master.server.cert',
               self.node_name + '.server.cert',
               self.node_name + '.server.key',
               self.node_name + '.client.cert',
               self.node_name + '.client.key' ]
        for f in fn:
            if not os.path.isfile(os.path.join(self.kpath, f)):
                return False
        return True

    def keystore_path(self, cert_name, suffix='.cert'):
        return os.path.join(self.kpath, cert_name + suffix)

    def get_server_context(self):
        #print("Security.get_server_context() for node %s" % self.node_name)
        if self.server_context_ is None:
            context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
            context.verify_mode = self.server_cert_verify_mode
            context.load_verify_locations(cafile=self.keystore_path('root','.cert'))
            stem = node_name_to_cert_stem(self.node_name)
            keyfile  = os.path.join(self.kpath, stem + '.server.key')
            certfile = os.path.join(self.kpath, stem + '.server.cert')
            if not os.path.isfile(keyfile) or not os.path.isfile(certfile):
                raise Exception("ahhhhhhHHHHHHHHHHHH where did all the certs go?")
            context.load_cert_chain(certfile, keyfile=keyfile)
            self.server_context_ = context
        return self.server_context_

    def get_client_context(self, server):
        #print("Security.get_client_context(%s) for node %s" % (server, self.node_name))
        if not server in self.client_contexts_:
            context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
            context.verify_mode = self.client_cert_verify_mode
            stem = node_name_to_cert_stem(self.node_name)
            context.load_verify_locations(cafile=self.keystore_path('root', '.cert'))
            keyfile  = os.path.join(self.kpath, stem + '.client.key')
            certfile = os.path.join(self.kpath, stem + '.client.cert')
            context.load_cert_chain(certfile, keyfile=keyfile)
            self.client_contexts_[server] = context
        return self.client_contexts_[server]

    def wrap_socket(self, sock, node_name):
        """
        Called whenever there is an opportunity to wrap a server socket
        """
        #print("SSLSecurity.wrap_socket() called for node %s" % node_name)
        context = self.get_server_context()
        return context.wrap_socket(sock, server_side=True)

    def xmlrpcapi(self, uri, node_name=None):
        #print("             SSLSecurity.xmlrpcapi(%s, %s)" % (uri, node_name or "[UNKNOWN]"))
        #if not node_name:
        #    print("      now i will call master.getCaller
        context = self.get_client_context(node_name or uri)
        st = XMLRPCTimeoutSafeTransport(context=context, timeout=10.0)
        return xmlrpcclient.ServerProxy(uri, transport=st, context=context)

    def xmlrpc_protocol(self):
        return 'https'

    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None):
        #print('SSLSecurity.connect() to node at %s:%d which is endpoint %s' % (dest_addr, dest_port, endpoint_id))
        # TODO: get certificates to talk to this URI
        context = self.get_client_context("%s:%d" % (dest_addr, dest_port))
        conn = context.wrap_socket(sock)
        try:
            conn.connect((dest_addr, dest_port))
        except Exception as e:
            print("  AHHH node %s SSLSecurity.connect() exception: %s" % (self.node_name, e))
            raise
        return conn

    def accept(self, server_sock, server_node_name):
        #print("SSLSecurity.accept(server_node_name=%s)" % server_node_name)
        context = self.get_server_context()
        (client_sock, client_addr) = server_sock.accept()
        client_stream = None
        try:
            client_stream = context.wrap_socket(client_sock, server_side=True)
        except Exception as e:
            print("SSLSecurity.accept() wrap_socket exception: %s" % e)
            raise
        return (client_stream, client_addr)

#########################################################################
_security = None
def init(node_name):
    #print('security.init(%s)' % node_name)
    global _security
    if _security is None:
        _logger.info("choosing security model...")
        if 'ROS_SECURITY' in os.environ:
            if os.environ['ROS_SECURITY'] == 'ssh':
                _security = SSHSecurity()
            elif os.environ['ROS_SECURITY'] == 'ssl' or os.environ['ROS_SECURITY'] == 'ssl_setup':
                _security = SSLSecurity(node_name)
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

###############################################################
# KEYSERVER STUFF
###############################################################

_ssl_certificate_creator = None

def keyserver_host():
    return 'localhost' # todo: not this

def keyserver_port():
    return 11310 # todo: not this

def keyserver_addr():
    return 'http://%s:%d' % (keyserver_host(), keyserver_port())

def keyserver_getCertificates(node_name):
    node_name = node_name_to_cert_stem(node_name) # sanitize and de-anonymize

    print('keyserver: getCertificates(%s)' % node_name)
    keypath = os.path.join(os.path.expanduser('~'), '.ros', 'keys', 'master')

    _ssl_certificate_creator.create_certs_if_needed(node_name)

    resp = {}
    fns = [ 'root.cert',
            'master.server.cert',
            node_name + '.server.cert',
            node_name + '.server.key',
            node_name + '.client.cert',
            node_name + '.client.key' ]
    for fn in fns:
        with open(os.path.join(keypath, fn), 'r') as f:
            resp[fn] = f.read()
    return resp

def keyserver_hello():
    return "I'm alive!"

def keyserver_main():
    global _ssl_certificate_creator
    from SimpleXMLRPCServer import SimpleXMLRPCServer
    from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler
    # todo: set this port to one below the ROS_MASTER_URI port
    _ssl_certificate_creator = SSLCertificateCreator()
    server = SimpleXMLRPCServer((keyserver_host(), keyserver_port()), SimpleXMLRPCRequestHandler, False)
    server.register_function(keyserver_getCertificates, 'getCertificates')
    server.register_function(keyserver_hello, 'hello') # it answers just to say it's alive
    print('entering xmlrpc_keyserver spin loop...')
    server.serve_forever()

def fork_xmlrpc_keyserver():
    print("forking an unsecured XML-RPC server to bootstrap SSL key distribution...")
    from multiprocessing import Process
    p = Process(target=keyserver_main)
    p.start()
    # spin until the keyserver is responding to requests
    keyserver_proxy = xmlrpcclient.ServerProxy(keyserver_addr())
    print("sleeping until keyserver has generated the initial keyring...")
    while True:
        try:
            keyserver_proxy.hello()
            break
        except Exception as e:
            time.sleep(0.01)
    print("horray, the keyserver is now open for business.")
 
