# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id: launch.py 2165 2008-09-17 22:38:49Z sfkwc $

"""
Defines the L{ROSLaunchConfig} object, which holds and the state of
the roslaunch file.
"""

import os
import logging
import types

import rospkg
import rospkg.distro
import rosgraph.names
import rosgraph.network

from .core import Master, local_machine, is_machine_local, RLException
import roslaunch.loader
import roslaunch.xmlloader

try:
    from rosmaster import DEFAULT_MASTER_PORT
except:
    DEFAULT_MASTER_PORT = 11311
    
def namespaces_of(name):
    """
    utility to determine namespaces of a name
    @raises ValueError
    @raises TypeError
    """
    if name is None: 
        raise ValueError('name')
    try:
        if not isinstance(name, basestring):
            raise TypeError('name')
    except NameError:
        if not isinstance(name, str):
            raise TypeError('name')
    if not name:
        return ['/']

    splits = [x for x in name.split('/') if x]
    return ['/'] + ['/'+'/'.join(splits[:i]) for i in range(1, len(splits))]

def get_roscore_filename():
    # precedence: look for version in /etc/ros.  If it's not there, fall back to roslaunch package
    filename = os.path.join(rospkg.get_etc_ros_dir(), 'roscore.xml')
    if os.path.isfile(filename):
        return filename
    r = rospkg.RosPack()
    return os.path.join(r.get_path('roslaunch'), 'resources', 'roscore.xml')

def load_roscore(loader, config, verbose=True):
    """
    Load roscore configuration into the ROSLaunchConfig using the specified XmlLoader
    @param config ROSLaunchConfig
    @param loader XmlLoader
    """
    f_roscore = get_roscore_filename()
    logging.getLogger('roslaunch').info('loading roscore config file %s'%f_roscore)            
    loader.load(f_roscore, config, core=True, verbose=verbose)    
        
def calculate_env_loader(env=None):
    """
    @raise RLException
    """
    if env is None:
        env = os.environ
    # guess the env loader based on distro name
    distro_name = rospkg.distro.current_distro_codename()
    # sanity check
    if distro_name in ['electric', 'diamondback', 'cturtle']:
        raise RLException("This version of roslaunch is not compatible with pre-Fuerte ROS distributions")
    return '/opt/ros/%s/env.sh'%(distro_name)

def _summary_name(node):
    """
    Generate summary label for node based on its package, type, and name
    """
    if node.name:
        return "%s (%s/%s)"%(node.name, node.package, node.type)
    else:
        return "%s/%s"%(node.package, node.type)
    
class ROSLaunchConfig(object):
    """
    ROSLaunchConfig is the container for the loaded roslaunch file state. It also
    is responsible for validating then executing the desired state. 
    """

    def __init__(self):
        """
        Initialize an empty config object. Master defaults to the environment's master.
        """
        self.master = Master()
        self.nodes_core = [] 
        self.nodes    = [] #nodes are unnamed
        
        self.roslaunch_files = [] # metadata about files used to create config
        
        # list of resolved node names. This is so that we can check for naming collisions
        self.resolved_node_names = []
        
        self.tests    = [] 
        self.machines = {} #key is name
        self.params   = {} #key is name
        self.clear_params = []
        self.executables = []

        # for tools like roswtf
        self.config_errors = []
        
        m = local_machine() #for local exec
        self.machines[m.name] = m
        self._assign_machines_complete = False
        self._remote_nodes_present = None

        self.logger = logging.getLogger('roslaunch')

    def add_roslaunch_file(self, f):
        """
        Add metadata about file used to create config
        """
        self.roslaunch_files.append(f)
        
    def add_config_error(self, msg):
        """
        Report human-readable error message related to configuration error
        @param msg: error message
        @type  msg: str
        """
        self.config_errors.append(msg)

    def set_master(self, m):
        """
        Set the master configuration
        @param m: Master            
        @type  m: L{Master}
        """
        self.master = m

    def has_remote_nodes(self):
        """
        @return: True if roslaunch will launch nodes on a remote machine
        @rtype: bool
        @raises: RLException
        """
        if not self._assign_machines_complete:
            raise RLException("ERROR: has_remote_nodes() cannot be called until prelaunch check is complete")
        return self._remote_nodes_present
    
    def assign_machines(self):
        """
        Assign nodes to machines and determine whether or not there are any remote machines
        """
        # don't repeat machine assignment
        if self._assign_machines_complete:
            return
        
        machine_unify_dict = {}
        
        self._assign_machines_complete = True
        # #653: current have to set all core nodes to local launch
        local_machine = self.machines['']
        for n in self.nodes_core:
            n.machine = local_machine

        #for n in self.nodes_core + self.nodes + self.tests:
        for n in self.nodes + self.tests:
            m = self._select_machine(n)
            
            # if machines have the same config keys it means that they are identical except
            # for their name. we unify the machine assignments so that we don't use
            # extra resources.
            config_key = m.config_key()
            if config_key in machine_unify_dict:
                new_m = machine_unify_dict[config_key]
                if m != new_m:
                    self.logger.info("... changing machine assignment from [%s] to [%s] as they are equivalent", m.name, new_m.name)
                    m = new_m
            else:
                machine_unify_dict[config_key] = m
            n.machine = m
            self.logger.info("... selected machine [%s] for node of type [%s/%s]", m.name, n.package, n.type)

        # determine whether or not there are any machines we will need
        # to setup remote roslaunch clients for
        self._remote_nodes_present = False 
        if [m for m in machine_unify_dict.values() if not is_machine_local(m)]:
            self._remote_nodes_present = True

    def summary(self, local=False):
        """
        Get a human-readable string summary of the launch
        @param local bool: if True, only print local nodes
        @return: summary
        @rtype: str
        """
        summary = '\nSUMMARY\n========'
        if self.clear_params:
            summary += '\n\nCLEAR PARAMETERS\n' + '\n'.join(sorted([' * %s'%p for p in self.clear_params]))
        if self.params:
            def strip_string(value):
                # not dealing with non-ascii characters here
                try:
                    value = str(value)
                except UnicodeEncodeError:
                    return '<...>'
                max_length = 20
                if len(value) > max_length:
                    value = value[:max_length - 3] + '...'
                # if non printable characters are present return replacement
                for i, char in enumerate(value):
                    o = ord(char)
                    if o < 32 or o > 126:
                        # skip when the special characters are only trailing whitespaces
                        value = value.rstrip()
                        if i >= len(value):
                            break
                        return '<...>'
                return value
            summary += '\n\nPARAMETERS\n' + '\n'.join(sorted([' * %s: %s' % (k, strip_string(v.value)) for k, v in self.params.items()]))
        if not local:
            summary += '\n\nMACHINES\n' + '\n'.join(sorted([' * %s'%k for k in self.machines if k]))
        summary += '\n\nNODES\n'
        namespaces = {}
        if local:
            nodes = [n for n in self.nodes if is_machine_local(n.machine)]
        else:
            nodes = self.nodes
        for n in nodes:
            ns = n.namespace
            if ns not in namespaces:
                namespaces[ns] = [n]
            else:
                namespaces[ns].append(n)
        for k,v in namespaces.items():
            summary += '  %s\n'%k + '\n'.join(sorted(['    %s'%_summary_name(n) for n in v]))
            summary += '\n'
        return summary

    def add_executable(self, exe):
        """
        Declare an exectuable to be run during the launch
        @param exe: Executable
        @type  exe: L{Executable}
        @raises ValueError
        """
        if not exe:
            raise ValueError("exe is None")
        self.executables.append(exe)
        
    def add_clear_param(self, param):
        """
        Declare a parameter to be cleared before new parameters are set
        @param param: parameter to clear
        @type  param: str
        """
        self.clear_params.append(param)

    def add_param(self, p, filename=None, verbose=True, override_params=True):
        """
        Declare a parameter to be set on the param server before launching nodes
        @param p: parameter instance
        @type  p: L{Param}
        """
        key = p.key

        # check for direct overrides
        if key in self.params and self.params[key] != p:
            if filename:
                self.logger.debug("[%s] overriding parameter [%s]"%(filename, p.key))
            else:
                self.logger.debug("overriding parameter [%s]"%p.key)
            if not override_params:
                return
        # check for parent conflicts
        for parent_key in [pk for pk in namespaces_of(key) if pk in self.params]:
            self.add_config_error("parameter [%s] conflicts with parent parameter [%s]"%(key, parent_key))

        self.params[key] = p
        if verbose:
            print("Added parameter [%s]" % key)
        t = type(p.value)
        if t in [bool, int, float]:
            self.logger.debug("add_param[%s]: type [%s] value [%s]"%(p.key, t, p.value))
        else:
            self.logger.debug("add_param[%s]: type [%s]"%(p.key, t))
            
    def add_machine(self, m, verbose=True):
        """
        Declare a machine and associated parameters so that it can be used for
        running nodes.
        @param m: machine instance
        @type  m: L{Machine}
        @return: True if new machine added, False if machine already specified.
        @rtype: bool
        @raises RLException: if cannot add machine as specified
        """
        name = m.name
        # Fuerte: all machines must have an env loader. We can guess
        # it from the distro name for easier migration.
        if not m.env_loader:
            m.env_loader = calculate_env_loader()
        if m.address == 'localhost': #simplify address comparison
            address = rosgraph.network.get_local_address()
            self.logger.info("addMachine[%s]: remapping localhost address to %s"%(name, address))
        if name in self.machines:
            if m != self.machines[name]:
                raise RLException("Machine [%s] already added and does not match duplicate entry"%name)
            return False
        else:
            self.machines[name] = m
            if verbose:
                print("Added machine [%s]" % name)
            return True

    def add_test(self, test, verbose=True):
        """
        Add test declaration. Used by rostest
        @param test: test node instance to add to launch
        @type  test: L{Test}
        """
        self.tests.append(test)

    def add_node(self, node, core=False, verbose=True):
        """
        Add node declaration
        @param node: node instance to add to launch
        @type  node: L{Node}
        @param core: if True, node is a ROS core node
        @type  core: bool
        @raises RLException: if ROS core node is missing required name
        """
        if node.name:
            # check for duplicates
            resolved_name = rosgraph.names.ns_join(node.namespace, node.name)
            matches = [n for n in self.resolved_node_names if n == resolved_name]
            if matches:
                raise RLException("roslaunch file contains multiple nodes named [%s].\nPlease check all <node> 'name' attributes to make sure they are unique.\nAlso check that $(anon id) use different ids."%resolved_name)
            else:
                self.resolved_node_names.append(resolved_name)
        
        if not core:
            self.nodes.append(node)
            if verbose:
                print("Added node of type [%s/%s] in namespace [%s]" % (node.package, node.type, node.namespace))
            self.logger.info("Added node of type [%s/%s] in namespace [%s]", node.package, node.type, node.namespace)
        else:
            if not node.name:
                raise RLException("ROS core nodes must have a name. [%s/%s]"%(node.package, node.type))
            self.nodes_core.append(node)
            if verbose:
                print("Added core node of type [%s/%s] in namespace [%s]" % (node.package, node.type, node.namespace))
            self.logger.info("Added core node of type [%s/%s] in namespace [%s]", node.package, node.type, node.namespace)
            
    def _select_machine(self, node):
        """
        Select a machine for a node to run on. For nodes that are
        already assigned to a machine, this will map the string name to
        a L{Machine} instance. If the node isn't already tagged with a
        particular machine, one will be selected for it.
        @param node: node to assign machine for
        @type  node: L{Node}
        @return: machine to run on
        @rtype: L{Machine}
        @raises RLException: If machine state is improperly configured
        """
        machine = node.machine_name
        #Lookup machine
        if machine:
            if not machine in self.machines:
                raise RLException("ERROR: unknown machine [%s]"%machine)
            return self.machines[machine]
        else:
            # assign to local machine
            return self.machines['']            

def load_config_default(roslaunch_files, port, roslaunch_strs=None, loader=None, verbose=False, assign_machines=True):
    """
    Base routine for creating a ROSLaunchConfig from a set of 
    roslaunch_files and or launch XML strings and initializing it. This
    config will have a core definition and also set the master to run
    on port.
    @param roslaunch_files: list of launch files to load
    @type  roslaunch_files: [str]
    @param port: roscore/master port override. Set to 0 or None to use default.
    @type  port: int
    @param roslaunch_strs: (optional) roslaunch XML strings to load
    @type  roslaunch_strs: [str]
    @param verbose: (optional) print info to screen about model as it is loaded. 
    @type  verbose: bool
    @param assign_machines: (optional) assign nodes to machines (default: True)
    @type  assign_machines: bool
    @return: initialized rosconfig instance
    @rytpe: L{ROSLaunchConfig} initialized rosconfig instance
    @raises: RLException
    """
    logger = logging.getLogger('roslaunch.config')
    
    # This is the main roslaunch server process. Load up the
    # files specified on the command line and launch the
    # requested resourcs.
            
    config = ROSLaunchConfig()
    if port:
        config.master.uri = rosgraph.network.create_local_xmlrpc_uri(port)

    loader = loader or roslaunch.xmlloader.XmlLoader()

    # load the roscore file first. we currently have
    # last-declaration wins rules.  roscore is just a
    # roslaunch file with special load semantics
    load_roscore(loader, config, verbose=verbose)

    # load the roslaunch_files into the config
    for f in roslaunch_files:
        try:
            logger.info('loading config file %s'%f)
            loader.load(f, config, verbose=verbose)
        except roslaunch.xmlloader.XmlParseException as e:
            raise RLException(e)
        except roslaunch.loader.LoadException as e:
            raise RLException(e)
        
    # we need this for the hardware test systems, which builds up
    # roslaunch launch files in memory
    if roslaunch_strs:
        for launch_str in roslaunch_strs:
            try:
                logger.info('loading config file from string')
                loader.load_string(launch_str, config)
            except roslaunch.xmlloader.XmlParseException as e:
                raise RLException('Launch string: %s\nException: %s'%(launch_str, e))
            except roslaunch.loader.LoadException as e:
                raise RLException('Launch string: %s\nException: %s'%(launch_str, e))

    # choose machines for the nodes
    if assign_machines:
        config.assign_machines()
    return config
    
