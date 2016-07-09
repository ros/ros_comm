from __future__ import print_function

import argparse
import os
import subprocess
import sys
import rosgraph.security as security
import rosgraph.keyserver as keyserver
import shutil


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
                if values not in security.GraphModes:
                    parser.error("Unknown MODE [{}] specified".format(values))
                setattr(namespace, self.dest, values)

        self.add_argument(
            '-k','--keyserver',
            action='store_true',
            help='enable keyserver')
        self.add_argument(
            '--config',
            action=CondAction,
            to_be_required=[self._option_string_actions['--keyserver']],
            help='path to custom config file')
        self.add_argument(
            '--keystore',
            action='store',
            help='path to custom keystore directory')
        graph_argument = self.add_argument(
            '-g','--graph',
            action='store',
            help='access control graph')
        self.add_argument(
            '-m','--mode',
            action=CondAction,
            to_be_required=[self._option_string_actions['--graph']],
            help='access control mode (audit|complain|[enforce]|train)')
        self.add_argument(
            '--version',
            action='version',
            version='%(prog)s 0.0')


def sroscore_main(argv = sys.argv):
    sroscore_parser = SroscoreParser(
        prog='sroscore',
        description='secure roscore')
    sroscore_parser.set()
    args, roscore_argv = sroscore_parser.parse_known_args(argv)

    if args.config is not None:
        os.environ['ROS_CONFIG_PATH'] = args.config
    if args.keystore is not None:
        os.environ['ROS_KEYSTORE_PATH'] = args.keystore

    if args.keyserver:
        # if we're in setup mode, we need to start an unsecured server that will
        # hand out the SSL certificates and keys so that nodes can talk to roscore
        os.environ['ROS_SECURITY'] = 'ssl_setup'

        if 'ROS_KEYSTORE_PATH' in os.environ:
            keys_dir = os.environ['ROS_KEYSTORE_PATH']
        else:
            keys_dir = os.path.join(os.path.expanduser('~'), '.ros', 'keys')

        if 'ROS_CONFIG_PATH' in os.environ:
            config_path = os.path.abspath(os.environ['ROS_CONFIG_PATH'])
        else:
            # config_path = os.path.join(os.path.expanduser('~'), '.ros', 'keys','sros_config.yaml')
            config_path = os.path.abspath('/home/ruffsl/sros/src/ruffsl/ros_comm/tools/sros/conf/sros_config.yaml')

        keyserver.fork_xmlrpc_keyserver(config_path, keys_dir)
    else:
        os.environ['ROS_SECURITY'] = 'ssl'

    if args.graph is not None:
        os.environ['ROS_GRAPH_NAME'] = args.graph
    if args.mode is not None:
        os.environ['ROS_GRAPH_MODE'] = args.mode
    else:
        os.environ['ROS_GRAPH_MODE'] = 'enforce'
    
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

