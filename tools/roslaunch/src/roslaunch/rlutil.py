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

"""
Uncategorized utility routines for roslaunch.

This API should not be considered stable.
"""

from __future__ import print_function

import os
import sys
import time

import roslib.packages

import rosclean
import rospkg
import rosgraph

import roslaunch.core
import roslaunch.config
import roslaunch.depends
from rosmaster import DEFAULT_MASTER_PORT

def check_log_disk_usage():
    """
    Check size of log directory. If high, print warning to user
    """
    try:
        d = rospkg.get_log_dir()
        roslaunch.core.printlog("Checking log directory for disk usage. This may take a while.\nPress Ctrl-C to interrupt") 
        disk_usage = rosclean.get_disk_usage(d)
        # warn if over a gig
        if disk_usage > 1073741824:
            roslaunch.core.printerrlog("WARNING: disk usage in log directory [%s] is over 1GB.\nIt's recommended that you use the 'rosclean' command."%d)
        else:
            roslaunch.core.printlog("Done checking log file disk usage. Usage is <1GB.")            
    except:
        pass

def resolve_launch_arguments(args):
    """
    Resolve command-line args to roslaunch filenames.

    :returns: resolved filenames, ``[str]``
    """

    # strip remapping args for processing
    args = rosgraph.myargv(args)
    
    # user can either specify:
    #  - filename + launch args
    #  - package + relative-filename + launch args
    if not args:
        return args
    resolved_args = None

    # try to resolve launch file in package first
    if len(args) >= 2:
        try:
            resolved = roslib.packages.find_resource(args[0], args[1])
            if len(resolved) > 1:
                raise roslaunch.core.RLException("multiple files named [%s] in package [%s]:%s\nPlease specify full path instead" % (args[1], args[0], ''.join(['\n- %s' % r for r in resolved])))
            if len(resolved) == 1:
                resolved_args = [resolved[0]] + args[2:]
        except rospkg.ResourceNotFound:
            pass
    # try to resolve launch file
    if resolved_args is None and (args[0] == '-' or os.path.isfile(args[0])):
        resolved_args = [args[0]] + args[1:]
    # raise if unable to resolve
    if resolved_args is None:
        if len(args) >= 2:
            raise roslaunch.core.RLException("[%s] is neither a launch file in package [%s] nor is [%s] a launch file name" % (args[1], args[0], args[0]))
        else:
            raise roslaunch.core.RLException("[%s] is not a launch file name" % args[0])
    return resolved_args

def _wait_for_master():
    """
    Block until ROS Master is online
    
    :raise: :exc:`RuntimeError` If unexpected error occurs
    """
    m = roslaunch.core.Master() # get a handle to the default master
    is_running = m.is_running()
    if not is_running:
        roslaunch.core.printlog("roscore/master is not yet running, will wait for it to start")
    while not is_running:
        time.sleep(0.1)
        is_running = m.is_running()
    if is_running:
        roslaunch.core.printlog("master has started, initiating launch")
    else:
        raise RuntimeError("unknown error waiting for master to start")

_terminal_name = None

def _set_terminal(s):
    import platform
    if platform.system() in ['FreeBSD', 'Linux', 'Darwin', 'Unix']:
        try:
            print('\033]2;%s\007'%(s))
        except:
            pass
    
def update_terminal_name(ros_master_uri):
    """
    append master URI to the terminal name
    """
    if _terminal_name:
        _set_terminal(_terminal_name + ' ' + ros_master_uri)

def change_terminal_name(args, is_core):
    """
    use echo (where available) to change the name of the terminal window
    """
    global _terminal_name
    _terminal_name = 'roscore' if is_core else ','.join(args)
    _set_terminal(_terminal_name)

def get_or_generate_uuid(options_runid, options_wait_for_master):
    """
    :param options_runid: run_id value from command-line or ``None``, ``str``
    :param options_wait_for_master: the wait_for_master command
      option. If this is True, it means that we must retrieve the
      value from the parameter server and need to avoid any race
      conditions with the roscore being initialized. ``bool``
    """

    # Three possible sources of the run_id:
    #
    #  - if we're a child process, we get it from options_runid
    #  - if there's already a roscore running, read from the param server
    #  - generate one if we're running the roscore
    if options_runid:
        return options_runid

    # #773: Generate a run_id to use if we launch a master
    # process.  If a master is already running, we'll get the
    # run_id from it instead
    param_server = rosgraph.Master('/roslaunch')
    val = None
    while val is None:
        try:
            val = param_server.getParam('/run_id')
        except:
            if not options_wait_for_master:
                val = roslaunch.core.generate_run_id()
    return val
    
def check_roslaunch(f, use_test_depends=False):
    """
    Check roslaunch file for errors, returning error message if check fails. This routine
    is mainly to support rostest's roslaunch_check.

    :param f: roslaunch file name, ``str``
    :param use_test_depends: Consider test_depends, ``Bool``
    :returns: error message or ``None``
    """
    try:
        rl_config = roslaunch.config.load_config_default([f], DEFAULT_MASTER_PORT, verbose=False)
    except roslaunch.core.RLException as e:
        return str(e)
    
    rospack = rospkg.RosPack()
    errors = []
    # check for missing deps
    try:
        base_pkg, file_deps, missing = roslaunch.depends.roslaunch_deps([f], use_test_depends=use_test_depends)
    except rospkg.common.ResourceNotFound as r:
        errors.append("Could not find package [%s] included from [%s]"%(str(r), f))
        missing = {}
        file_deps = {}
    except roslaunch.substitution_args.ArgException as e:
        errors.append("Could not resolve arg [%s] in [%s]"%(str(e), f))
        missing = {}
        file_deps = {}
    for pkg, miss in missing.items():
        # even if the pkgs is not found in packges.xml, if other package.xml provdes that pkgs, then it will be ok
        all_pkgs = []
        try:
            for file_dep in file_deps.keys():
                file_pkg = rospkg.get_package_name(file_dep)
                all_pkgs.extend(rospack.get_depends(file_pkg,implicit=False))
                miss_all = list(set(miss) - set(all_pkgs))
        except Exception as e:
            print(e, file=sys.stderr)
            miss_all = True
        if miss_all:
            roslaunch.core.printerrlog("Missing package dependencies: %s/package.xml: %s"%(pkg, ', '.join(miss)))
            errors.append("Missing package dependencies: %s/package.xml: %s"%(pkg, ', '.join(miss)))
        elif miss:
            roslaunch.core.printerrlog("Missing package dependencies: %s/package.xml: %s (notify upstream maintainer)"%(pkg, ', '.join(miss)))
    
    # load all node defs
    nodes = []
    for filename, rldeps in file_deps.items():
        nodes.extend(rldeps.nodes)

    # check for missing packages
    for pkg, node_type in nodes:
        try:
            rospack.get_path(pkg)
        except:
            errors.append("cannot find package [%s] for node [%s]"%(pkg, node_type))

    # check for missing nodes
    for pkg, node_type in nodes:
        try:
            if not roslib.packages.find_node(pkg, node_type, rospack=rospack):
                errors.append("cannot find node [%s] in package [%s]"%(node_type, pkg))
        except Exception as e:
            errors.append("unable to find node [%s/%s]: %s"%(pkg, node_type, str(e)))
                
    # Check for configuration errors, #2889
    for err in rl_config.config_errors:
        errors.append('ROSLaunch config error: %s' % err)

    if errors:
        return '\n'.join(errors)
                          
def print_file_list(roslaunch_files):
    """
    :param roslaunch_files: list of launch files to load, ``str``

    :returns: list of files involved in processing roslaunch_files, including the files themselves.
    """
    from roslaunch.config import load_config_default, get_roscore_filename
    import roslaunch.xmlloader
    try:
        loader = roslaunch.xmlloader.XmlLoader(resolve_anon=True)
        config = load_config_default(roslaunch_files, None, loader=loader, verbose=False, assign_machines=False)
        files = [os.path.abspath(x) for x in set(config.roslaunch_files) - set([get_roscore_filename()])]
        print('\n'.join(files))
    except roslaunch.core.RLException as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

