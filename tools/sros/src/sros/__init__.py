from __future__ import print_function

import argparse
import os
import subprocess
import sys
import rosgraph.security as security
import rosgraph.policy as policy
import rosgraph.keyserver as keyserver
import shutil
import rospkg


class SroscoreParser(argparse.ArgumentParser):
    """Argument parser class sroscore"""

    def set(self):
        """Setup parser for sroscore"""

        class CondAction(argparse.Action):
            def __init__(self, option_strings, dest, nargs=None, **kwargs):
                x = kwargs.pop('to_be_required', [])
                super(CondAction, self).__init__(option_strings, dest, **kwargs)
                self.make_required = x
                self.dest = dest

            def __call__(self, parser, namespace, values, option_string=None):
                for x in self.make_required:
                    x.required = True
                if values not in policy.GraphModes:
                    parser.error("Unknown MODE [{}] specified".format(values))
                setattr(namespace, self.dest, values)

        self.add_argument(
            '-k','--keyserver',
            action='store_true',
            help='enable keyserver')
        self.add_argument(
            '--keyserver_config',
            action=CondAction,
            to_be_required=[self._option_string_actions['--keyserver']],
            help='path to custom config file')
        self.add_argument(
            '--keyserver_mode',
            action=CondAction,
            to_be_required=[self._option_string_actions['--keyserver']],
            help='verify mode *on* keyserver (CERT_NONE|[CERT_OPTIONAL]|CERT_REQUIRED)')
        self.add_argument(
            '--keyserver_verify',
            action=CondAction,
            to_be_required=[self._option_string_actions['--keyserver']],
            help='verify mode *of* keyserver (CERT_NONE|CERT_OPTIONAL|[CERT_REQUIRED])')
        self.add_argument(
            '--keystore_path',
            action='store',
            help='path to custom keystore directory path')
        self.add_argument(
            '--policy_config',
            action='store',
            help='policy config for access control')
        self.add_argument(
            '--policy_mode',
            action=CondAction,
            # to_be_required=[self._option_string_actions['--policy_config']],
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

def sroscore_main(argv = sys.argv):
    sroscore_parser = SroscoreParser(
        prog='sroscore',
        description='secure roscore')
    sroscore_parser.set()
    args, roscore_argv = sroscore_parser.parse_known_args(argv)

    check_set_environ(
        'SROS_KEYSERVER_CONFIG',
        args.keyserver_config,
        os.path.join(os.path.expanduser('~'), '.ros', 'sros', 'keyserver_config.yaml'))
    
    check_set_environ(
        'SROS_KEYSTORE_PATH',
        args.keystore_path,
        os.path.join(os.path.expanduser('~'), '.ros', 'keys'))
    
    check_set_environ(
        'SROS_KEYSERVER_VERIFY',
        args.keyserver_verify,
        'CERT_REQUIRED')
    
    check_set_environ(
        'SROS_KEYSERVER_MODE',
        args.keyserver_mode,
        'CERT_OPTIONAL')

    check_set_environ(
        'SROS_POLICY_CONFIG',
        args.policy_config,
        os.path.join(os.path.expanduser('~'), '.ros', 'sros', 'policy_config.yaml'))
    
    check_set_environ(
        'SROS_POLICY_MODE',
        args.policy_mode,
        'enforce')

    if args.keyserver:
        # if we're in setup mode, we need to start an unsecured server that will
        # hand out the SSL certificates and keys so that nodes can talk to roscore
        os.environ['SROS_SECURITY'] = 'ssl_setup'
        os.environ['SROS_POLICY'] = 'namespace'
        
        keyserver_config = os.path.abspath(os.environ['SROS_KEYSERVER_CONFIG'])
        keystore_path = os.path.abspath(os.environ['SROS_KEYSTORE_PATH'])
        keyserver_mode = os.environ['SROS_KEYSERVER_MODE']
        if not os.path.isfile(keyserver_config):
            keyserver_config_default = os.path.join(rospkg.get_etc_ros_dir(), 'keyserver_config.yaml')
            shutil.copy(keyserver_config_default, keyserver_config)

        policy_config = os.path.abspath(os.environ['SROS_POLICY_CONFIG'])
        if not os.path.isfile(policy_config):
            policy_config_default = os.path.join(rospkg.get_etc_ros_dir(), 'policy_config.yaml')
            shutil.copy(policy_config_default, policy_config)
        
        keyserver.fork_xmlrpc_keyserver(keyserver_config, keystore_path, keyserver_mode)
    else:
        os.environ['SROS_SECURITY'] = 'ssl'
        os.environ['SROS_POLICY'] = 'namespace'
    
    import roslaunch
    roslaunch.main(['roscore', '--core'] + roscore_argv[1:])


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

    if args.subparser_name == 'keys':
        return sros_keys(args)


def sros_keys(args):
    if args.delete:
        if args.all:
            return sros_keys_delete(args)


def sros_keys_delete(args):
    # todo: parameterize keystore location, if we expand to multiple keyrings
    keypath = os.path.join(os.path.expanduser('~'), '.ros', 'keys')
    # do a tiny bit of sanity check...
    if keypath.count('/') <= 1:
        print("woah. I probably shouldn't delete this path: [%s]" % keypath)
        sys.exit(1)
    print("deleting keystore directory: [%s]" % keypath)
    shutil.rmtree(keypath)
    print("done.")
    return 0

