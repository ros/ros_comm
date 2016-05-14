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
    def xmlrpcapi(self, uri):
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
                print(" need to create a tunnel!")
                # first, ask the OS for a free port to listen on
                port_probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                port_probe.bind(('', 0))
                local_port = port_probe.getsockname()[1]
                port_probe.close() # the OS won't re-allocate this port anytime soon
                ssh_incantation = ["ssh", "-nNT", "-L", "%d:localhost:%d" % (local_port, dest_port), str(dest_hostname)]
                print("spawning ssh tunnel with: %s" % " ".join(ssh_incantation))
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
            print("spawning ssh tunnel with: %s" % " ".join(ssh_incantation))
            try:
                subprocess.Popen(ssh_incantation, shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                time.sleep(1) # HAXX. do a retry-loop
                sock.connect(('127.0.0.1', local_port))
            except Exception as e:
                print("OH NOES couldn't connect through tunnel: %s" % str(e))
                raise

#########################################################################
class SSLSecurity(Security):
    def __init__(self):
        super(SSLSecurity, self).__init__()
        _logger.info("  rospy.security.SSLSecurity init")
        self.kpath = os.path.join(os.path.expanduser('~'), '.ros', 'keys')
        self.openssl_conf_path = os.path.join(self.kpath, 'openssl.conf')
        self.root_ca_path = os.path.join(self.kpath, 'root.ca')
        self.root_cert_path = os.path.join(self.kpath, 'root.cer')

        # TODO: be able to change this, for "strict mode" at some point
        self.cert_verify_mode = ssl.CERT_OPTIONAL

        self.server_context_ = None
        self.client_contexts_ = {}
        self.certs = {}

    def node_name_to_cert_stem(self, node_name):
        # TODO: convert anonymous names into something consistent, so we
        # don't end up generating arbitrary numbers of them
        stem = node_name
        if (stem[0] == '/'):
            stem = stem[1:]
        return stem.replace('/', '__')

    def request_cert(self, node_name, mode):
        print("request_cert(%s, %s)" % (node_name, mode))
        stem = self.node_name_to_cert_stem(node_name)
        cert_name = "%s.%s" % (node_name, mode)
        if not cert_name in self.certs:
            print("requesting certificate %s from master..." % cert_name)
            master_proxy = rosgraph.masterapi.Master(node_name)
            response = master_proxy.getCertificates(node_name)
            #print("getCertificates response: %s" % repr(response))
            # todo: error checking would be good
            self.certs[cert_name] = response['server.cert']
        return self.certs[cert_name]

    def create_context(self, mode, node_name):
        print("Security.create_context(%s, %s)" % (mode, node_name))
        context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        context.verify_mode = self.cert_verify_mode
        context.load_verify_locations(cafile=self.root_cert_path)
        if mode != 'server' and mode != 'client':
            raise ValueError("unknown SSL context mode: [%s]" % mode)
        keyfile  = os.path.join(self.kpath, node_name + '.' + mode + '.key')
        certfile = os.path.join(self.kpath, node_name + '.' + mode + '.cert')
        if not os.path.isfile(keyfile) or not os.path.isfile(certfile):
            self.request_cert(node_name, mode)
        print("loading %s and %s" % (certfile, keyfile))
        context.load_cert_chain(certfile, keyfile=keyfile)
        return context

    def get_server_context(self, node_name):
        # TODO: request root CA and cert if we don't already have them
        if self.server_context_ is None:
            self.server_context_ = self.create_context('server', node_name)
        return self.server_context_

    def get_client_context(self, node_name):
        # TODO: request root CA and cert if we don't already have them
        print("Security.get_client_context(%s)" % node_name)
        if not node_name in self.client_contexts_:
            #self.client_contexts_[node_name] = self.create_context('server', node_name)
            print("creating client context for %s" % node_name)
            context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
            context.verify_mode = self.cert_verify_mode
            context.load_verify_locations(cafile=self.root_cert_path)
            #keyfile  = os.path.join(self.kpath, node_name + '.' + mode + '.key')
            #certfile = os.path.join(self.kpath, node_name + '.' + mode + '.cert')
            #if not os.path.isfile(keyfile) or not os.path.isfile(certfile):
            #    self.request_cert(node_name, mode)
            #print("loading %s and %s" % (certfile, keyfile))
            #context.load_cert_chain(certfile, keyfile=keyfile)
            self.client_contexts_[node_name] = context
            # TODO: add our client certificate here to allow the server to authenticate us
        return self.client_contexts_[node_name]

    def create_certs_if_needed(self):
        if not os.path.exists(self.kpath):
            print("creating keys path: %s" % self.kpath)
            os.makedirs(self.kpath)
        self.root_key_path = os.path.join(self.kpath, 'root.key')
        if not os.path.isfile(self.root_cert_path) or \
           not os.path.isfile(self.root_key_path):
            openssl_incantation = "openssl req -batch -newkey rsa:4096 -subj /C=US/ST=CA/O=ros -days 3650 -x509 -sha256 -nodes -out %s -keyout %s" % (self.root_cert_path, self.root_key_path)
            print("creating root certificate and key: %s" % openssl_incantation)
            subprocess.check_call(openssl_incantation.split(' '))

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
extendedKeyUsage = serverAuth
#subjectAltName = @alt_names
#
#[ alt_names ]
#DNS.1 = localhost
'''.format(self.root_cert_path, root_db_path, self.root_key_path, root_serial_path))
        self.create_cert('master','server')
        self.create_cert('master','client')
        self.create_cert('roslaunch','server')
        self.create_cert('roslaunch','client')

    def create_cert(self, node_name, suffix):
        if not names.is_legal_base_name(node_name):
            raise ValueError("security.create_certs() received illegal node name: %s" % node_name)
        _logger.info("creating certificate for node [%s] as %s" % (node_name, suffix))
        cert_name = node_name + '.' + suffix
        csr_path = os.path.join(self.kpath, '%s.csr' % cert_name)
        key_path = os.path.join(self.kpath, '%s.key' % cert_name)
        if not os.path.isfile(csr_path):
            openssl_incantation = r'openssl req -batch -subj /C=US/ST=CA/O=ros/CN={0} -newkey rsa:4096 -sha256 -nodes -out {1} -keyout {2}'.format(cert_name, csr_path, key_path)
            print("creating master certificate signing request and key: %s" % openssl_incantation)
            subprocess.check_call(openssl_incantation.split(' '))

        cert_path = os.path.join(self.kpath, '%s.cert' % cert_name)
        if not os.path.isfile(cert_path):
            openssl_incantation = "openssl ca -batch -config {0} -notext -in {1} -out {2}".format(self.openssl_conf_path, csr_path, cert_path)
            print("creating master certificate: %s" % openssl_incantation)
            subprocess.check_call(openssl_incantation.split(' '))

        # TODO: for python >= 2.7.9, create an SSL context which will
        # cache ssl sessions to speed up repeated connections
        #try:
            #self.server_context = ssl.create_default_context(purpose=Purpose.CLIENT_AUTH, cafile=self.root_ca_path)
        #except Exception as e:
        #    print("OH NOES couldn't create SSL context: %s" % str(e))
        #    raise


    def wrap_socket(self, sock, node_name):
        """
        Called whenever there is an opportunity to wrap a server socket
        """

        print("SSLSecurity.wrap_socket() called for node %s" % node_name)
        if not names.is_legal_base_name(node_name):
            raise ValueError("security.wrap_socket() received illegal node name: %s" % node_name)

        #if (node_name == 'roslaunch'):
            # TODO: not sure how to handle this, since roscore fires 
            # off a roslaunch xmlrpc server to bring up master, so it's
            # kind of a chicken-and-egg. Maybe special-case this socket
            # so it can only receive from localhost?
        #    return sock

        if node_name == 'master' or node_name == 'roslaunch':
            self.create_certs_if_needed()

        context = self.get_server_context(node_name)
        return context.wrap_socket(sock, server_side=True)

    def xmlrpcapi(self, uri, node_name):
        print("SSLSecurity.xmlrpcapi(%s, %s)" % (uri, node_name))
        context = self.get_client_context(node_name)
        return xmlrpcclient.ServerProxy(uri, context=context)

    def xmlrpc_protocol(self):
        return 'https'

    def cert_path(self, node_name, mode):
        path = os.path.join(self.kpath, '%s.%s.cert' % (node_name, mode))
        if not os.path.isfile(path):
            self.create_cert(node_name, mode)
        return path

    def key_path(self, node_name, mode):
        path = os.path.join(self.kpath, '%s.%s.key' % (node_name, mode))
        if not os.path.isfile(path):
            self.create_cert(node_name, mode)
        return path

    def getCertificates(self, caller_id, node_name):
        """
        This function is only called by the Master XML-RPC handler, and only
        should be available during "permissive mode." A strict deployment
        will/should disable this function to prevent unauthorized creation or
        distribution of certificates and keys.
        """
        print("security.getCertificates(caller_id=%s, node_name=%s)" % (caller_id, node_name))
        # first, transform the node name into a legal certificate name stem
        stem = self.node_name_to_cert_stem(node_name)
        print('   using stem = [%s]' % stem)
        response = { }
        with open(self.cert_path(stem, 'server'), 'r') as f:
            response['server.cert'] = f.read()
        with open(self.cert_path(stem, 'client'), 'r') as f:
            response['client.cert'] = f.read()
        if caller_id == node_name:
            with open(self.key_path(stem, 'server'), 'r') as f:
                response['server.key'] = f.read()
            with open(self.key_path(stem, 'client'), 'r') as f:
                response['client.key'] = f.read()
        return response

    def connect(self, sock, dest_addr, dest_port, endpoint_id, pub_node_name, timeout=None):
        print('SSLSecurity.connect() to node %s at %s:%d which is endpoint %s' % (pub_node_name, dest_addr, dest_port, endpoint_id))
        context = self.get_client_context(pub_node_name)
        conn = context.wrap_socket(sock)
        try:
            conn.connect((dest_addr, dest_port))
        except Exception as e:
            print("SSLSecurity.connect() exception: %s" % e)
            raise
        return conn

    def accept(self, server_sock, server_node_name):
        print("SSLSecurity.accept(server_node_name=%s)" % server_node_name)
        context = self.get_server_context(server_node_name)
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
def get_security():
    global _security
    _logger.info("security.get_security()")
    if _security is None:
        _logger.info("choosing security model...")
        if 'ROS_SECURITY' in os.environ:
            if os.environ['ROS_SECURITY'] == 'ssh':
                _security = SSHSecurity()
            elif os.environ['ROS_SECURITY'] == 'ssl':
                _security = SSLSecurity()
            else:
                raise ValueError("unknown ROS security model: [%s]" % os.environ['ROS_SECURITY'])
        else:
            _security = NoSecurity()
    return _security
