from __future__ import print_function

import os
import subprocess
import sys
import rosgraph.security as security

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

def sros_main(argv = sys.argv):
    print(repr(argv))
    #if len(argv) 
