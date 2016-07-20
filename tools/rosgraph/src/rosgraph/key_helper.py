#!/usr/bin/python

from __future__ import print_function
from cryptography import hazmat, x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import rsa, dsa, ec

from OpenSSL import crypto, SSL
import os
import shutil
import sys
import yaml
import getpass
import datetime
import uuid
import rospkg

from sros_consts import ROLE_STRUCT

from copy import deepcopy

from rosgraph_helper import GraphStructure

class KeyBlob:
    '''
    This class is used to load or generate keys and certificates with openssl
    '''
    def __init__(self, name, config):
        '''
        Initializes and sets class attributes
        :param name: string of key name
        :param config: dict of key configuration
        '''
        self.name = name
        self.config = config
        self.password = None
        self.cert_path = None
        self.key_path = None
        self.cert = None
        self.key = None


    def _add_extensions(self, cert_builder, ca_blob):
        '''
        Adds extensions to certificate
        :param cert_builder: x509.CertificateBuilder used to add extensions to
        :param ca_blob: KeyBlob of ca used when extension may need issuer's cert
        :return: cert_builder; x509.CertificateBuilder with added extensions
        '''

        if self.config['cert']['x509_extensions'] is not None:
            # self._sort_extension_logic()
            x509_extensions = self.config['cert']['x509_extensions']

            if ca_blob is None:
                issuer_public_key = self.key.public_key()
            else:
                issuer_public_key = ca_blob.cert.public_key()

            extension_name = 'AuthorityKeyIdentifier'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    extension_type = getattr(x509, extension_name)
                    extension = extension_type.from_issuer_public_key(issuer_public_key)
                    critical = x509_extensions[extension_name]['critical']
                    cert_builder = cert_builder.add_extension(extension, critical)

            extension_name = 'BasicConstraints'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    extension_type = getattr(x509, extension_name)
                    extension = extension_type(**x509_extensions[extension_name]['value'])
                    critical = x509_extensions[extension_name]['critical']
                    cert_builder = cert_builder.add_extension(extension, critical)

            extension_name = 'ExtendedKeyUsage'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    value = x509_extensions[extension_name]['value']
                    if value is not None:
                        usages = []
                        for oid in value:
                            usage = getattr(x509.oid.ExtendedKeyUsageOID, oid, None)
                            if not usage:
                                usage = x509.ObjectIdentifier(oid)
                            usages.append(usage)
                        extension_type = getattr(x509, extension_name)
                        extension = extension_type(usages)
                        critical = x509_extensions[extension_name]['critical']
                        cert_builder = cert_builder.add_extension(extension, critical)

            extension_name = 'KeyUsage'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    extension_type = getattr(x509, extension_name)
                    kwargs = {}
                    kwargz = ['digital_signature',
                              'content_commitment',
                              'key_encipherment',
                              'data_encipherment',
                              'key_agreement',
                              'key_cert_sign',
                              'crl_sign',
                              'encipher_only',
                              'decipher_only']
                    for kwarg in kwargz: kwargs[kwarg] = False
                    if x509_extensions[extension_name]['value'] is not None:
                        kwargs.update(x509_extensions[extension_name]['value'])
                    extension = extension_type(**kwargs)
                    critical = x509_extensions[extension_name]['critical']
                    cert_builder = cert_builder.add_extension(extension, critical)

            extension_name = 'NameConstraints'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    extension_type = getattr(x509, extension_name)
                    permitted_subtrees = x509_extensions[extension_name]['value']['permitted_subtrees']
                    excluded_subtrees  = x509_extensions[extension_name]['value']['excluded_subtrees']
                    if permitted_subtrees:
                        for i, v in enumerate(permitted_subtrees):
                            permitted_subtrees[i] = x509.UniformResourceIdentifier(unicode(v))
                    if excluded_subtrees:
                        for i, v in enumerate(excluded_subtrees):
                            excluded_subtrees[i] = x509.UniformResourceIdentifier(unicode(v))
                    if permitted_subtrees or excluded_subtrees:
                        extension = extension_type(permitted_subtrees, excluded_subtrees)
                    else:
                        permitted_subtrees = None
                        excluded_subtrees  = [x509.UniformResourceIdentifier(unicode("**"))]
                        extension = extension_type(permitted_subtrees, excluded_subtrees)
                    critical = x509_extensions[extension_name]['critical']
                    cert_builder = cert_builder.add_extension(extension, critical)

            extension_name = 'SubjectKeyIdentifier'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    extension_type = getattr(x509, extension_name)
                    extension = extension_type.from_public_key(self.key.public_key())
                    critical = x509_extensions[extension_name]['critical']
                    cert_builder = cert_builder.add_extension(extension, critical)

            extension_name = 'SubjectAlternativeName'
            if extension_name in x509_extensions:
                if x509_extensions[extension_name] is not None:
                    extension_type = getattr(x509, extension_name)
                    value = x509_extensions[extension_name]['value']
                    general_names = []
                    for v in value:
                        general_names.append(x509.UniformResourceIdentifier(unicode(v)))
                    extension = extension_type(general_names)
                    critical = x509_extensions[extension_name]['critical']
                    cert_builder = cert_builder.add_extension(extension, critical)

        return cert_builder


    def _generate_cert_builder(self):
        '''
        Generates X509 certificate builder and applies subject and expiration info
        :return: cert_builder; x509.CertificateBuilder with basic subject and valid-dates added
        '''

        cert_config = self.config['cert']

        attributes = []
        for attribute_key in cert_config['subject']:
            oid = getattr(NameOID, attribute_key)
            value = unicode(cert_config['subject'][attribute_key])
            attribute = x509.NameAttribute(oid, value)
            attributes.append(attribute)
        subject = x509.Name(attributes)

        utcnow = datetime.datetime.utcnow()
        if isinstance(cert_config['not_valid_before'], int):
            not_before_datetime = utcnow + datetime.timedelta(seconds=cert_config['not_valid_before'])
        else:
            not_before_datetime = cert_config['not_valid_before']
        if isinstance(cert_config['not_valid_after'], int):
            not_after_datetime = utcnow + datetime.timedelta(seconds=cert_config['not_valid_after'])
        else:
            not_after_datetime = cert_config['not_valid_after']

        if cert_config['serial_number'] is None:
            cert_config['serial_number'] = int(uuid.uuid4())

        cert_builder = x509.CertificateBuilder().subject_name(
            subject
        ).serial_number(
            cert_config['serial_number']
        ).not_valid_before(
            not_before_datetime
        ).not_valid_after(
            not_after_datetime
        ).public_key(
            self.key.public_key()
        )

        return cert_builder


    def generate_key(self):
        '''
        Generates key pair using type and length specified in config
        :return: None
        '''

        if self.config['key']['key_type'] == 'rsa':
            self.key = rsa.generate_private_key(
                public_exponent=65537,
                key_size=self.config['key']['key_peram'],
                backend=default_backend()
            )
        elif self.config['key']['key_type'] == 'dsa':
            self.key = dsa.generate_private_key(
                key_size=self.config['key']['key_peram'],
                backend=default_backend()
            )
        elif self.config['key']['key_type'] == 'ec':
            self.key = ec.generate_private_key(
                curve=getattr(ec, self.config['key']['key_peram'])(),
                backend=default_backend()
            )
        else:
            raise ValueError("\nFailed to generate key, no key_type key type provided!\n"
                             "Offending key name: {}\n".format(self.name))


    def _get_fingerprint_algorithm(self):
        '''
        Returns the fingerprint algorithm object type to use as defined in the config
        :return: fingerprint_algorithm
        '''
        if self.config['fingerprint_algorithm'] is not None:
            fingerprint_algorithm = getattr(hashes, self.config['fingerprint_algorithm'])()
            return fingerprint_algorithm
        else:
            raise ValueError("\nNo fingerprint algorithm is specified!\n"
                             "Offending key name: {}\n".format(self.name))


    def create_cert(self, ca_blob=None):
        '''
        Create certificate and singe using key pair
        :param ca_blob: KeyBlob used and CA, when None the certificate will be self singed
        :return: None
        '''

        cert_builder = self._generate_cert_builder()

        if ca_blob is None:
            self.cert = self._add_extensions(
                cert_builder.issuer_name(cert_builder._subject_name), ca_blob
            ).sign(self.key, self._get_fingerprint_algorithm(), default_backend())
        else:
            self.cert = self._add_extensions(
                cert_builder.issuer_name(ca_blob.cert.subject), ca_blob
            ).sign(ca_blob.key, ca_blob._get_fingerprint_algorithm(), default_backend())


    def dump_cert(self):
        '''
        Get PEM Encoded certificate serialization
        @return: serialization of certificate
        '''
        return self.cert.public_bytes(serialization.Encoding.PEM)


    def save_cert(self, cert_path=None):
        '''
        Save certificate to disk
        :param cert_path: full certificate file path to write to
        :return: None
        '''
        if cert_path is None:
            cert_path = self.cert_path
        with open(cert_path, "wb") as f:
            f.write(self.dump_cert())


    def dump_key(self):
        '''
        Get PEM Encoded/Encrypted key serialization
        @return: serialization of key
        '''
        if self.config['encryption_algorithm'] is None:
            encryption_algorithm = serialization.NoEncryption()
        else:
            encryption_algorithm = getattr(serialization, self.config['encryption_algorithm'])(self.password)

        return self.key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=encryption_algorithm,
            )


    def save_key(self, key_path=None):
        '''
        Save private key to disk
        :param key_path: full key file path to write to
        :return: None
        '''
        if key_path is None:
            key_path = self.key_path

        with open(key_path, "wb") as f:
            f.write(self.dump_key())


    def load_cert(self, cert_path=None):
        '''
        Load certificate from disk
        :param cert_path: full certificate file path to load from
        :return: None
        '''
        if cert_path is None:
            cert_path = self.cert_path
        with open(cert_path, 'rb') as f:
            self.cert = x509.load_pem_x509_certificate(f.read(), default_backend())


    def load_key(self, key_path=None):
        '''
        Load private key from disk
        :param key_path: full key file path to load from
        :return: None
        '''
        if key_path is None:
            key_path = self.key_path
        with open(key_path, 'rb') as f:
            self.key = serialization.load_pem_private_key(
                f.read(),
                password = self.password,
                backend = default_backend())


    def init_password(self):
        '''
        Get password either from matching environment variable or promt from user input.
        Only does so if encryption_algorithm has been specified.
        :param env: name of environment variable to check for password
        :return: None
        '''
        encryption_algorithm = self.config['encryption_algorithm']
        if encryption_algorithm is not None:
            password_env = self.config['password_env']
            if password_env is not None:
                self.password = os.environ[password_env]
            else:
                while (True):
                    password = getpass.getpass(prompt='Enter pass phrase for {}: '.format(self.name), stream=None)
                    password = getpass.getpass(prompt='Verifying - Enter pass phrase for {}: '.format(self.name),
                                                  stream=None)
                    if (password == password):
                        break
                self.password = password
        else:
            self.password = None


    def check_keys_match(self):
        '''
        Check if public and private keys are a valid key pair
        :return: True if key pairs match, False otherwise
        '''
        return self.key.public_key().public_numbers() == self.cert.public_key().public_numbers()


class KeyHelper:
    def __init__(self, config_path, keys_dir):
        '''
        Initializes and sets class attributes
        :param name: string of key name
        :param config: dict of key configuration
        '''

        # print("config_path: ", config_path)
        # print("keys_dir: ", keys_dir)

        self.config_path = config_path
        self.config = load_config(config_path)
        self.keys_dir = keys_dir

    def init_ca(self):
        '''
        Initializes Certificate Authority
        @return:
        '''
        config = self.config
        keys_dir = self.keys_dir

        master_name = "master"
        master_config = config['keys'][master_name]
        master_dir = os.path.join(keys_dir, master_name)
        master_blob = KeyBlob(master_name, master_config)
        self.keys = dict()

        if (config['keys'][master_name]['issuer_name'] is None):
            get_keys(master_dir, master_blob)
            self.keys[master_name] = master_blob

        elif (config['keys'][master_name]['issuer_name'] in config['keys']):
            root_name = config['keys']['master']['issuer_name']
            root_config = config['keys'][root_name]
            root_blob = KeyBlob(root_name, root_config)
            root_dir = os.path.join(keys_dir, root_name)
            get_keys(root_dir, root_blob)
            print("Certificate generated: {}".format(root_name))
            self.keys[root_name] = root_blob

            get_keys(master_dir, master_blob, root_blob)
            print("Certificate generated: {}".format(master_name))
            self.keys[master_name] = master_blob

        hash_dir = os.path.join(keys_dir, 'capath')
        rehash(hash_dir, self.keys, clean=True)

    def init_keyserver(self):
        '''
        Initializes keystore certificate
        @return: 
        '''
        config = self.config
        keys_dir = self.keys_dir

        keyserver_name = 'keyserver'
        keyserver_config = config['keys'][keyserver_name]
        keyserver_dir = os.path.join(keys_dir, keyserver_name)
        keyserver_blob = KeyBlob(keyserver_name, keyserver_config)
        ca_blob = self.keys[keyserver_config['issuer_name']]

        get_keys(keyserver_dir, keyserver_blob, ca_blob)
        # print("Certificate generated: {}".format(keyserver_name))

        return keyserver_blob.cert_path, keyserver_blob.key_path

    def get_nodestore(self, node_stem):
        '''
        get certificate response
        @return:
        '''
        config_path = self.config_path
        config = self.config
        #TODO: use security.node_stem_to_node_name for this
        node_name = node_stem.split('/')[-1]

        key_config = config['keys']['nodes']

        resp = {'topics':{}, 'services':{}, 'parameters':{}}

        for role_name, role_struct in ROLE_STRUCT.iteritems():
            for mode_name, mode_struct in role_struct.iteritems():
                key_mode = role_name + '.' + mode_name
                key_name = node_name + '.' + key_mode
                node_config = deepcopy(key_config)
                node_config['key_mode'] = key_mode
                if 'cert' in node_config:
                    node_config['cert']['subject']['COMMON_NAME'] = key_name
                    graph_path = os.path.join(os.path.dirname(config_path), node_config['cert']['graph_path'])
                else:
                    graph_path = None
                node_config = extention_parsing(node_stem, node_config, graph_path)
                node_blob = KeyBlob(os.path.basename(key_name), node_config)
                get_keys(None, node_blob, self.keys[node_blob.config['issuer_name']])
                # self.keys[key_name] = node_blob
                # print("Certificate generated: {}".format(key_name))
                resp[role_name][node_blob.name + '.pem'] = node_blob.dump_key()
                resp[role_name][node_blob.name + '.cert'] = node_blob.dump_cert()

        return resp

    def get_ca(self):
        '''
        get Certificate Authorities
        @return:
        '''
        hash_dir = os.path.join(self.keys_dir, 'capath')
        resp = get_hash_certs(hash_dir)
        return resp


def get_hash_certs(hash_dir):
    '''
    Get dect of certs with hashed names
    @param hash_dir:
    @return:
    '''
    certs = {}
    for cert_file in os.listdir(hash_dir):
        cert_path = os.path.join(hash_dir, cert_file)
        with open(cert_path, 'rb') as f:
            cert = x509.load_pem_x509_certificate(f.read(), default_backend())
        certs[cert_file] = cert.public_bytes(encoding=serialization.Encoding.PEM)

    return certs


def check_path(path):
    '''
    Check for path, and create it if non existing
    :param path:
    :return: None
    '''
    if not os.path.exists(path):
        os.makedirs(path)


def load_config(path):
    '''
    Load and parse configuration file
    :param path: file path to configuration file
    :return: dict representation of config structure
    '''
    with open(path, 'r') as stream:
        config = yaml.load(stream)
    return config


def get_keys(key_dir, key_blob, ca_blob=None):
    '''
    Loads or creates and save keys and certificates using initialized KeyBlob and with specified CA
    loading or creating is determined by config
    :param key_dir: folder path to store keys and certificates
    :param key_blob: KeyBlob used to generate and save keys and certificates
    :param ca_blob: KeyBlob of CA used to sign generated certificates
    :return: None
    '''
    if key_dir is not None:
        check_path(key_dir)
        key_blob.cert_path = os.path.join(key_dir, key_blob.name + '.cert')
        key_blob.key_path  = os.path.join(key_dir, key_blob.name + '.pem')
        over_write_cert = not os.path.exists(key_blob.cert_path)
        over_write_key = not os.path.exists(key_blob.key_path)
    else:
        over_write_cert = False
        over_write_key = False

    # over_write_cert = key_blob.config['key'] is not None
    # over_write_key = key_blob.config['cert'] is not None

    if key_dir:
        if over_write_key:
            key_blob.generate_key()
            key_blob.init_password()
            key_blob.save_key()
        else:
            if 'encryption_algorithm' in key_blob.config:
                key_blob.init_password()
            key_blob.load_key()
    else:
        key_blob.generate_key()
        if 'encryption_algorithm' in key_blob.config:
            key_blob.init_password()

    if key_dir:
        if over_write_cert:
            key_blob.create_cert(ca_blob)
            key_blob.save_cert()
        else:
            key_blob.load_cert()
    else:
        key_blob.create_cert(ca_blob)

    if not key_blob.check_keys_match():
        raise ValueError("\nFailed to load certificate, does not match private key!\n"
                         "New key pair was generated, public keys from old certificates do not match.\n"
                         "Offending cert: {}\n".format(key_blob.cert_path) +
                         "Offending key: {}\n".format(key_blob.key_path))


def rehash(hash_dir, keys_dict, clean=False):
    '''
    Rehash given keys and create symbolic links to CA certificate within given directory
    :param hash_dir: path to directory to create symbolic links
    :param keys_dict: dict of KeBlobs create symbolic links for
    :param clean: bool used to delete and thus clean hash_dir
    :return: None
    '''
    if os.path .exists(hash_dir) and clean:
        shutil.rmtree(hash_dir)
    check_path(hash_dir)
    hash_list = []
    for key_name, key_blob in keys_dict.iteritems():
        crypto_cert = crypto.load_certificate(crypto.FILETYPE_PEM, key_blob.cert.public_bytes(serialization.Encoding.PEM))
        subject_name_hash = crypto_cert.subject_name_hash()
        hash = format(subject_name_hash, '02x')
        hash_dict = {'hash':hash,
                     'link_path':os.path.join(hash_dir, hash + '.0'),
                     'cert_path':key_blob.cert_path,
                     'key_name':key_name}
        if os.path.exists(hash_dict['link_path']):
            os.unlink(hash_dict['link_path'])
        hash_list.append(hash_dict)

    for hash_dict in hash_list:
        try:
            os.symlink(hash_dict['cert_path'], hash_dict['link_path'])
        except:
            raise ValueError("\nSubject Name Hashes from your certs are colliding!\n"
                             "Please make sure all CA certificates subjects are unique!\n"
                             "In no particular order...\n"
                             "Offending cert: {}\n".format(hash_dict['key_name']) +
                             "Offending hash: {}\n".format(hash_dict['hash']))


def get_policy_constraints(node_stem, node_config, graph):
    policy_type, mode_name = node_config['key_mode'].split('.')
    mask_type = ROLE_STRUCT[policy_type][mode_name]['mask']

    applicable_nodes = graph.filter_nodes(node_stem)
    if applicable_nodes:
        applicable_policies = graph.filter_policies(applicable_nodes, policy_type)
        if applicable_policies:
            applicable_allow = graph.filter_modes(applicable_policies, 'allow')
            applicable_deny  = graph.filter_modes(applicable_policies, 'deny')

            allow_subtrees = graph.filter_masks(applicable_allow, mask_type)
            deny_subtrees  = graph.filter_masks(applicable_deny,  mask_type)

            permitted_subtrees = graph.list_policy_namespaces(allow_subtrees)
            excluded_subtrees  = graph.list_policy_namespaces(deny_subtrees)

            if not permitted_subtrees:
                permitted_subtrees = None
            if not excluded_subtrees:
                excluded_subtrees = None

            applicable_name_spaces = {
                'permitted_subtrees': permitted_subtrees,
                'excluded_subtrees':  excluded_subtrees,
            }
            return applicable_name_spaces

    applicable_name_spaces = {
        'permitted_subtrees': None,
        'excluded_subtrees' : None
    }
    return applicable_name_spaces


def set_extention(extension_name, extensions_config, node_config, default):
    extension = extensions_config[extension_name]
    if 'value' in extension:
        value = extension['value']
        if value is None:
            extension['value'] = default
        elif isinstance(value, str):
            extension['value'] = node_config[value]
        elif not isinstance(value, list):
            raise ValueError("\nImproper value for {} extension specified in config!\n".format(extension_name))


def extention_parsing(node_stem, node_config, graph_path):
    role_name, mode_name = node_config['key_mode'].split('.')
    mode_types = ROLE_STRUCT[role_name][mode_name]['OID']

    if node_config['cert']['x509_extensions'] is not None:
        extensions_config = node_config['cert']['x509_extensions']

        extension_name = 'SubjectAlternativeName'
        if extension_name in extensions_config:
            set_extention(extension_name, extensions_config, node_config, [node_stem])

        extension_name = 'ExtendedKeyUsage'
        if extension_name in extensions_config:
            set_extention(extension_name, extensions_config, node_config, mode_types)

        extension_name = 'NameConstraints'
        if extension_name in extensions_config:
            if graph_path is not None:
                graph = GraphStructure()
                graph.load_graph(graph_path)
                default = get_policy_constraints(node_stem, node_config, graph)
            else:
                default = None
            set_extention(extension_name, extensions_config, node_config, default)

    return node_config