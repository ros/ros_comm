from __future__ import print_function

import argparse
import os
import subprocess
import sys
import rosgraph.sros_consts as sros_consts
import rosgraph.keyserver as keyserver
import shutil
import rospkg

class SroscoreParser(argparse.ArgumentParser):
    """Argument parser class sroscore"""

    def set(self):
        """Setup parser for sroscore"""

        self.add_argument(
            '--keystore_path',
            action='store',
            default=rospkg.get_sros_keystore_path(),
            help='path to custom keystore path')
        self.add_argument(
            '--keyserver_verify',
            action='store',
            help='verify mode *of* keyserver (CERT_NONE|CERT_OPTIONAL|[CERT_REQUIRED])')
        self.add_argument(
            '--policy_config',
            action='store',
            help='policy config path for access control')
        self.add_argument(
            '--policy_mode',
            action='store',
            help='policy mode for access control (audit|complain|[enforce]|train)')
        self.add_argument(
            '--version',
            action='version',
            version='%(prog)s 0.0')

def check_set_environ(name, arg, default):
    if arg is not None:
        os.environ[name] = arg
    else:
        os.environ[name] = default

def check_environ(name, default):
    if name not in os.environ:
        os.environ[name] = default

def sroscore_main(argv = sys.argv):
    sroscore_parser = SroscoreParser(
        prog='sroscore',
        description='secure roscore')
    sroscore_parser.set()
    args, roscore_argv = sroscore_parser.parse_known_args(argv)

    check_set_environ(
        'SROS_KEYSTORE_PATH',
        args.keystore_path,
        rospkg.get_sros_keystore_path())
    
    check_set_environ(
        'SROS_KEYSERVER_VERIFY',
        args.keyserver_verify,
        'CERT_REQUIRED')

    check_set_environ(
        'SROS_POLICY_CONFIG',
        args.policy_config,
        os.path.join(rospkg.get_sros_config_dir(), 'policy_config.yaml'))
    
    check_set_environ(
        'SROS_POLICY_MODE',
        args.policy_mode,
        'enforce')

    check_environ(
        'SROS_SECURITY',
        sros_consts.TLSv1_2)    
    check_environ(
        'SROS_POLICY',
        'namespace')
    
    import roslaunch
    roslaunch.main(['roscore', '--core'] + roscore_argv[1:])


def sroslaunch_main(argv = sys.argv):

    check_environ(
        'SROS_KEYSTORE_PATH',
        rospkg.get_sros_keystore_path())
    check_environ(
        'SROS_KEYSERVER_VERIFY',
        'CERT_REQUIRED')
    check_environ(
        'SROS_SECURITY',
        sros_consts.TLSv1_2)    
    check_environ(
        'SROS_POLICY',
        'namespace')
    
    import roslaunch
    roslaunch.main(argv)

class SroskeyserverParser(argparse.ArgumentParser):
    """Argument parser class sroskeyserver"""

    def set(self):
        """Setup parser for sroscore"""

        self.add_argument(
            '-c', '--keyserver_config',
            action='store',
            default=os.path.join(rospkg.get_sros_config_dir(), 'keyserver_config.yaml'),
            help='path to custom config file')
        self.add_argument(
            '-m', '--keyserver_mode',
            action='store',
            default='CERT_OPTIONAL',
            help='verify mode *on* keyserver (CERT_NONE|[CERT_OPTIONAL]|CERT_REQUIRED)')
        self.add_argument(
            '-k', '--keystore_path',
            action='store',
            default=rospkg.get_sros_keystore_path(),
            help='path to custom keystore path')
        self.add_argument(
            '-p', '--port',
            action='store',
            default=0,
            help='keyserver port')
        self.add_argument(
            '-v', '--keyserver_verify',
            action='store',
            help='verify mode *of* keyserver (CERT_NONE|CERT_OPTIONAL|[CERT_REQUIRED])')
        self.add_argument(
            '--version',
            action='version',
            version='%(prog)s 0.0')

def sroskeyserver_main(argv = sys.argv):
    sroskeyserver_parser = SroskeyserverParser(
        prog='sroskeyserver',
        description='secure ros keyserver')
    sroskeyserver_parser.set()
    args, argv = sroskeyserver_parser.parse_known_args(argv)

    check_set_environ(
        'SROS_KEYSERVER_VERIFY',
        args.keyserver_verify,
        'CERT_REQUIRED')
    check_environ(
        'SROS_SECURITY',
        sros_consts.TLSv1_2)
    check_environ(
        'SROS_POLICY',
        'namespace')

    port = keyserver.DEFAULT_KEYSERVER_PORT
    if args.port:
        port = int(args.port)

    keystore_path = os.path.abspath(args.keystore_path)
    keyserver_mode = keyserver.check_verify_mode(args.keyserver_mode)

    keyserver_config = os.path.abspath(args.keyserver_config)
    if not os.path.isfile(keyserver_config):
        keyserver_config_default = os.path.join(rospkg.get_etc_ros_dir(), 'keyserver_config.yaml')
        if not os.path.exists(os.path.dirname(keyserver_config)):
            os.makedirs(os.path.dirname(keyserver_config))
        shutil.copy(keyserver_config_default, keyserver_config)

        policy_config = os.path.join(rospkg.get_sros_config_dir(), 'policy_config.yaml')
        if not os.path.isfile(policy_config):
            policy_config_default = os.path.join(rospkg.get_etc_ros_dir(), 'policy_config.yaml')
            if not os.path.exists(os.path.dirname(policy_config)):
                os.makedirs(os.path.dirname(policy_config))
            shutil.copy(policy_config_default, policy_config)

    keyserver.start_keyserver(keyserver_config, keystore_path, keyserver_mode, port)

class SrosParser(argparse.ArgumentParser):
    """Argument parser class sros"""

    def set(self):
        """Setup parser for sros"""

        # create the top-level parser
        subparsers = self.add_subparsers(help='help for subcommand', dest='subparser_name')

        # create the parser for the "key" command
        key_subparser = subparsers.add_parser(
            'keys',
            help='keys --help')
        key_subparser.add_argument(
            '-D', '--delete',
            action='store_true',
            help='delete keys')
        key_subparser.add_argument(
            '-A', '--all',
            action='store_true',
            help='all keys')


def sros_main(argv = sys.argv):
    sros_parser = SrosParser(
        prog='sros',
        description="sros tools")
    sros_parser.set()
    args = sros_parser.parse_args(argv[1:])

    if args.subparser_name == 'keystore':
        return sros_keystore(args)


def sros_keystore(args):
    if args.delete:
        if args.all:
            return sros_keystore_delete(args)


def sros_keystore_delete(args):
    # todo: parameterize keystore location, if we expand to multiple keyrings
    keystore_path = rospkg.get_sros_keystore_path()
    # do a tiny bit of sanity check...
    if keystore_path.count('/') <= 1:
        print("woah. I probably shouldn't delete this path: [%s]" % keystore_path)
        sys.exit(1)
    print("deleting keystore directory: [%s]" % keystore_path)
    shutil.rmtree(keystore_path)
    print("done.")
    return 0

