from __future__ import print_function

import logging
import os
import names
import time
import socket
import subprocess
import traceback
import ssl
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
    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None):
        try:
            _logger.info('connecting to ' + str(dest_addr)+ ' ' + str(dest_port))
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

    def xmlrpcapi(self, uri):
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

    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None):
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

    def create_certs_if_needed(self):
        if not os.path.exists(self.kpath):
            print("creating keys path: %s" % self.kpath)
            os.makedirs(self.kpath)
        self.root_cert_path = os.path.join(self.kpath, 'root.cer')
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
keyUsage = digitalSignature,keyEncipherment
extendedKeyUsage = serverAuth
'''.format(self.root_cert_path, root_db_path, self.root_key_path, root_serial_path))
        self.create_cert('master','server')
        self.create_cert('master','client')

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

        if (node_name == 'roslaunch'):
            # TODO: not sure how to handle this, since roscore fires 
            # off a roslaunch xmlrpc server to bring up master, so it's
            # kind of a chicken-and-egg. Maybe special-case this socket
            # so it can only receive from localhost?
            return sock

        if (node_name == 'master'):
            self.create_certs_if_needed()

        keyfile  = os.path.join(self.kpath, node_name + '.server.key' )
        certfile = os.path.join(self.kpath, node_name + '.server.cert')

        # TODO: request root CA if we don't already have it
        
        cert_reqs = ssl.CERT_OPTIONAL # TODO: this needs to be an option somehow; this is what distinguishes "learning mode" from "strict mode"
        ssl_version = ssl.PROTOCOL_TLSv1 # TODO: update as we move to newer distros
        return ssl.wrap_socket(sock, keyfile=keyfile, certfile=certfile,
            server_side=True, cert_reqs = cert_reqs,
            ca_certs = self.root_ca_path,
            ssl_version = ssl_version)

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
