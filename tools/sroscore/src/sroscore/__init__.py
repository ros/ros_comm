from __future__ import print_function

import os
import subprocess
import sys
import rosgraph.security as security

def roscore_main(argv = sys.argv):
    if not 'ROS_SECURITY' in os.environ:
        os.environ['ROS_SECURITY'] = 'ssl_setup' # default: ssl setup mode
    sm = os.environ['ROS_SECURITY']
    # creating root and master certificates if they don't exist
    if sm == 'ssl' or sm == 'ssl_setup':
        security.ssl_bootstrap()
    # if we're in setup mode, we need to start an ftp server that will
    # hand out our ca.cert and master.server.cert files
    if sm == 'ssl_setup':
        security.fork_ftp_cert_server()
    
    import roslaunch
    roslaunch.main(['roscore', '--core'] + sys.argv[1:])
