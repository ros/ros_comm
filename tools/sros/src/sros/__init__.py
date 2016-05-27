from __future__ import print_function

import os
import subprocess
import sys
import rosgraph.security as security
import shutil

def roscore_main(argv = sys.argv):
    if not 'ROS_SECURITY' in os.environ:
        os.environ['ROS_SECURITY'] = 'ssl_setup' # default: ssl setup mode
    sm = os.environ['ROS_SECURITY']
    # if we're in setup mode, we need to start an unsecured server that will
    # hand out the SSL certificates and keys so that nodes can talk to roscore
    if sm == 'ssl_setup':
        security.fork_xmlrpc_keyserver()
    
    import roslaunch
    roslaunch.main(['roscore', '--core'] + sys.argv[1:])

def usage(subcmd=None):
    if subcmd == 'keys':
        print("""
usage: sros keys SUBCOMMAND [OPTIONS]

where SUBCOMMAND is one of:
  delete\n""")
    else:
        print("""
usage: sros COMMAND [OPTIONS]

where COMMAND is one of:
  keys

command-specific help can be obtained by:

sros COMMAND --help\n""")
        return 1

def sros_main(argv = sys.argv):
    if len(argv) <= 1 or argv[1] == '--help':
        return usage()
    cmd = argv[1]
    if cmd == 'keys':
        return sros_keys(argv)
    else:
        return usage()

def sros_keys(argv):
    if len(argv) <= 2 or argv[2] == '--help':
        return usage('keys')
    subcmd = argv[2]
    if subcmd == 'delete':
        return sros_keys_delete(argv)
    else:
        return usage('keys')

def sros_keys_delete(argv):
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

