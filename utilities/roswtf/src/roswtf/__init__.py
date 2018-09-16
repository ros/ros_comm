# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
roswtf command-line tool.
"""

from __future__ import print_function

import os
import socket
import sys
import traceback

import rospkg
import rosgraph.names
    
def yaml_results(ctx):
    cd = ctx.as_dictionary()
    d = {}
    d['warnings'] = {}
    d['errors'] = {}
    wd = d['warnings']
    for warn in ctx.warnings:
        wd[warn.format_msg%cd] = warn.return_val
    ed = d['warnings']        
    for err in ctx.warnings:
        ed[err.format_msg%cd] = err.return_val
    import yaml
    print(yaml.dump(d))

def print_results(ctx):
    if not ctx.warnings and not ctx.errors:
        print("No errors or warnings")
    else:
        if ctx.warnings:
            print("Found %s warning(s).\nWarnings are things that may be just fine, but are sometimes at fault\n" % len(ctx.warnings))
            for warn in ctx.warnings:
                print('\033[1mWARNING\033[0m', warn.msg)
            print('')

        if ctx.errors:
            print("Found %s error(s).\n"%len(ctx.errors))
            for e in ctx.errors:
                print('\033[31m\033[1mERROR\033[0m', e.msg)
                #print("ERROR:", e.msg
    
def roswtf_main():
    try:
        import std_msgs.msg
        import rosgraph_msgs.msg
    except ImportError:
        print("ERROR: The core ROS message libraries (std_msgs and rosgraph_msgs) have not been built.")
        sys.exit(1)
    
    from roswtf.context import WtfException
    try:
        _roswtf_main()
    except WtfException as e:
        print(str(e), file=sys.stderr)
        
def _roswtf_main():
    launch_files = names = None
    # performance optimization
    rospack = rospkg.RosPack()
    all_pkgs = rospack.list()

    import optparse
    parser = optparse.OptionParser(usage="usage: roswtf [launch file]", description="roswtf is a tool for verifying a ROS installation and running system. Checks provided launchfile if provided, else current stack or package.")
    # #2268
    parser.add_option("--all", 
                      dest="all_packages", default=False,
                      action="store_true",
                      help="run roswtf against all packages")
    # #2270
    parser.add_option("--no-plugins", 
                      dest="disable_plugins", default=False,
                      action="store_true",
                      help="disable roswtf plugins")

    parser.add_option("--offline", 
                      dest="offline", default=False,
                      action="store_true",
                      help="only run offline tests")

    #TODO: --all-pkgs option
    options, args = parser.parse_args()
    if args:
        launch_files = args
        if 0:
            # disable names for now as don't have any rules yet
            launch_files = [a for a in args if os.path.isfile(a)]
            names = [a for a in args if not a in launch_files]
            names = [rosgraph.names.script_resolve_name('/roswtf', n) for n in names]

    from roswtf.context import WtfContext
    from roswtf.environment import wtf_check_environment, invalid_url, ros_root_check
    from roswtf.graph import wtf_check_graph
    import roswtf.rosdep_db
    import roswtf.py_pip_deb_checks
    import roswtf.network
    import roswtf.packages
    import roswtf.roslaunchwtf
    import roswtf.stacks    
    import roswtf.plugins
    if not options.disable_plugins:
        static_plugins, online_plugins = roswtf.plugins.load_plugins()
    else:
        static_plugins, online_plugins = [], []
        
    # - do a ros_root check first and abort if it fails as rest of tests are useless after that
    error = ros_root_check(None, ros_root=os.environ['ROS_ROOT'])
    if error:
        print("ROS_ROOT is invalid: "+str(error))
        sys.exit(1)

    all_warnings = []
    all_errors = []
    
    if launch_files:
        ctx = WtfContext.from_roslaunch(launch_files)
        #TODO: allow specifying multiple roslaunch files
    else:
        curr_package = rospkg.get_package_name('.')
        if curr_package:
            print("Package:", curr_package)
            ctx = WtfContext.from_package(curr_package)
            #TODO: load all .launch files in package
        elif os.path.isfile('stack.xml'):
            curr_stack = os.path.basename(os.path.abspath('.'))
            print("Stack:", curr_stack)
            ctx = WtfContext.from_stack(curr_stack)
        else:
            print("No package or stack in the current directory")
            ctx = WtfContext.from_env()
        if options.all_packages:
            print("roswtf will run against all packages")
            ctx.pkgs = all_pkgs

    # static checks
    wtf_check_environment(ctx)
    roswtf.rosdep_db.wtf_check(ctx)
    roswtf.py_pip_deb_checks.wtf_check(ctx)
    roswtf.network.wtf_check(ctx)
    roswtf.packages.wtf_check(ctx)
    roswtf.stacks.wtf_check(ctx)    
    roswtf.roslaunchwtf.wtf_check_static(ctx)
    for p in static_plugins:
        p(ctx)

    print("="*80)
    print("Static checks summary:\n")
    print_results(ctx)

    # Save static results and start afresh for online checks
    all_warnings.extend(ctx.warnings)
    all_errors.extend(ctx.errors)
    del ctx.warnings[:]
    del ctx.errors[:]    

    # test online
    print("="*80)

    try:

        if options.offline or not ctx.ros_master_uri or invalid_url(ctx.ros_master_uri) or not rosgraph.is_master_online():
            online_checks = False
        else:
            online_checks = True
        if online_checks:
            online_checks = True
            print("Beginning tests of your ROS graph. These may take awhile...")
            
            # online checks
            wtf_check_graph(ctx, names=names)
        elif names:
            # TODO: need to rework this logic
            print("\nCannot communicate with master, unable to diagnose [%s]"%(', '.join(names)))
            return
        else:
            print("\nROS Master does not appear to be running.\nOnline graph checks will not be run.\nROS_MASTER_URI is [%s]"%(ctx.ros_master_uri))
            return

        # spin up a roswtf node so we can subscribe to messages
        import rospy
        rospy.init_node('roswtf', anonymous=True)

        online_checks = True
        roswtf.roslaunchwtf.wtf_check_online(ctx)

        for p in online_plugins:
            online_checks = True
            p(ctx)

        if online_checks:
            # done
            print("\nOnline checks summary:\n")
            print_results(ctx)
            
    except roswtf.context.WtfException as e:
        print(str(e), file=sys.stderr)
        print("\nAborting checks, partial results summary:\n")
        print_results(ctx)
    except Exception as e:
        traceback.print_exc()
        print(str(e), file=sys.stderr)
        print("\nAborting checks, partial results summary:\n")
        print_results(ctx)

    #TODO: print results in YAML if run remotely
    #yaml_results(ctx)
