from __future__ import print_function
import rospy
import os
import time
import socket
import subprocess
import traceback
from rospy.exceptions import TransportInitError

try:
    import urllib.parse as urlparse #Python 3.x
except ImportError:
    import urlparse


try:
    import xmlrpc.client as xmlrpcclient #Python 3.x
except ImportError:
    import xmlrpclib as xmlrpcclient #Python 2.x

#########################################################################

class Security(object):
    # TODO: add security logging stuff here
    def __init__(self):
        rospy.loginfo("security init")

#########################################################################

class NoSecurity(Security):
    def __init__(self):
        super(NoSecurity, self).__init__()
        rospy.loginfo("  rospy.security.NoSecurity init")
    def xmlrpcapi(self, uri):
        uriValidate = urlparse.urlparse(uri)
        if not uriValidate[0] or not uriValidate[1]:
            return None
        return xmlrpcclient.ServerProxy(uri)
    def connect(self, sock, dest_addr, dest_port, endpoint_id, timeout=None):
        try:
            rospy.loginfo('connecting to ' + str(dest_addr)+ ' ' + str(dest_port))
            sock.connect((dest_addr, dest_port))
        except TransportInitError as tie:
            rospyerr("Unable to initiate TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))
            raise
        except Exception as e:
            rospy.logwarn("Unknown error initiating TCP/IP socket to %s:%s (%s): %s"%(dest_addr, dest_port, endpoint_id, traceback.format_exc()))
            # TODO: figure out how to bubble up and trigger the full close/release behavior in TCPROSTransport
            sock.close() # no reconnection as error is unknown
            raise TransportInitError(str(e)) #re-raise i/o error

#########################################################################

class SSHSecurity(Security):
    def __init__(self):
        super(SSHSecurity, self).__init__()
        rospy.loginfo("  rospy.security.SSHSecurity init")
        self.ssh_tunnels = { }

    def xmlrpcapi(self, uri):
        uriValidate = urlparse.urlparse(uri)
        if not uriValidate[0] or not uriValidate[1]:
            return None
        requested_uri = uri
        rospy.loginfo("rospy.security.xmlrpcapi(%s) host = %s" % (uri, uriValidate.hostname))

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
        rospy.loginfo("remapped xmlrpcapi request: %s => %s" % (requested_uri, uri))
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
        rospy.loginfo("  rospy.security.SSLSecurity init")
        self.create_certs_if_needed()

    def create_certs_if_needed(self):
        print("checking if there is a root certificate available...")
        kpath = os.path.join(os.path.expanduser('~'), '.ros', 'keys')
        if not os.path.exists(kpath):
            print("creating keys path: %s" % kpath)
            os.makedirs(kpath)
        root_cert_path = os.path.join(kpath, 'root.cer')
        root_privkey_path = os.path.join(kpath, 'root.privkey.pem')
        if not os.path.isfile(root_cert_path):
            openssl_incantation = "openssl req -batch -newkey rsa:4096 -subj /C=US/ST=CA/O=ros -days 3650 -x509 -sha256 -nodes -out %s -keyout %s" % (root_cert_path, root_privkey_path)
            print("creating root certificate and key: %s" % openssl_incantation)
            subprocess.check_call(openssl_incantation.split(' '))

        root_db_path = os.path.join(kpath, 'certindex')
        if not os.path.isfile(root_db_path):
            open(root_db_path, 'w').close() # creates an empty file

        root_serial_path = os.path.join(kpath, 'serial')
        if not os.path.isfile(root_serial_path):
            with open(root_serial_path, 'w') as f:
                f.write('000a')

        openssl_conf_path = os.path.join(kpath, 'openssl.conf')
        if not os.path.isfile(openssl_conf_path):
            with open(openssl_conf_path, 'w') as f:
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
'''.format(root_cert_path, root_db_path, root_privkey_path, root_serial_path))

        roscore_csr_path = os.path.join(kpath, 'roscore.csr')
        roscore_key_path = os.path.join(kpath, 'roscore.key')
        if not os.path.isfile(roscore_csr_path):
            openssl_incantation = r'openssl req -batch -subj /C=US/ST=CA/O=ros/CN=roscore -newkey rsa:4096 -sha256 -nodes -out {0} -keyout {1}'.format(roscore_csr_path, roscore_key_path)
            print("creating roscore certificate signing request and key: %s" % openssl_incantation)
            subprocess.check_call(openssl_incantation.split(' '))

        roscore_cert_path = os.path.join(kpath, 'roscore.cert')
        if not os.path.isfile(roscore_cert_path):
            openssl_incantation = "openssl ca -batch -config {0} -notext -in {1} -out {2}".format(openssl_conf_path, roscore_csr_path, roscore_cert_path)
            print("creating roscore certificate: %s" % openssl_incantation)
            subprocess.check_call(openssl_incantation.split(' '))


#########################################################################
_security = None
def get_security():
    global _security
    rospy.loginfo("security.get_security()")
    if _security is None:
        rospy.loginfo("choosing security model...")
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
