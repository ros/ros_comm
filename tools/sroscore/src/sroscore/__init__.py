from __future__ import print_function

import os
import subprocess
import sys
import rospy

def check_security_model():
    if not 'ROS_SECURITY' in os.environ:
        print("\033[91mWOAH THERE! You asked to run a secure ROS command, but ROS_SECURITY is not defined in the environment. You can set the ROS_SECURITY on the command line when running a program, like this:\n\nROS_SECURITY=ssl rosmaster\n\nor this:\n\nROS_SECURITY=ssl roscore\033[0m")
        sys.exit(1)

def rosmaster_main(argv = sys.argv):
    check_security_model()
    rospy.security.get_security() # will create certs if needed
    import rosmaster
    rosmaster.rosmaster_main()

def roscore_main(argv = sys.argv):
    check_security_model()
    rospy.security.get_security() # will create certs if needed
    import roslaunch
    roslaunch.main(['roscore', '--core'] + sys.argv[1:])
