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
# Revision $Id$

"""
Local process implementation for running and monitoring nodes.
"""

import os
import signal
import subprocess 
import time
import traceback

import rospkg

from roslaunch.core import *
#from roslaunch.core import setup_env
from roslaunch.node_args import create_local_process_args
from roslaunch.pmon import Process, FatalProcessLaunch

from rosmaster.master_api import NUM_WORKERS

import logging
_logger = logging.getLogger("roslaunch")

_TIMEOUT_SIGINT  = 15.0 #seconds
_TIMEOUT_SIGTERM = 2.0 #seconds

_counter = 0
def _next_counter():
    global _counter
    _counter += 1
    return _counter

def create_master_process(run_id, type_, ros_root, port, num_workers=NUM_WORKERS, timeout=None, close_sockets=False):
    """
    Launch a master
    @param type_: name of master executable (currently just Master.ZENMASTER)
    @type  type_: str
    @param ros_root: ROS_ROOT environment setting
    @type  ros_root: str
    @param port: port to launch master on
    @type  port: int
    @param num_workers: number of worker threads.
    @type  num_workers: int
    @param timeout: socket timeout for connections.
    @type  timeout: float
    @param close_sockets: enable close CLOSE_WAIT sockets.
    @type close_sockets: bool
    @raise RLException: if type_ or port is invalid
    """    
    if port < 1 or port > 65535:
        raise RLException("invalid port assignment: %s"%port)

    _logger.info("create_master_process: %s, %s, %s, %s, %s, %s", type_, ros_root, port, num_workers, timeout, close_sockets)
    # catkin/fuerte: no longer use ROS_ROOT-relative executables, search path instead
    master = type_
    # zenmaster is deprecated and aliased to rosmaster
    if type_ in [Master.ROSMASTER, Master.ZENMASTER]:        
        package = 'rosmaster'        
        args = [master, '--core', '-p', str(port), '-w', str(num_workers)]
        if timeout is not None:
            args += ['-t', str(timeout)]
        if close_sockets:
            args += ['--close-sockets']
    else:
        raise RLException("unknown master typ_: %s"%type_)

    _logger.info("process[master]: launching with args [%s]"%args)
    log_output = False
    return LocalProcess(run_id, package, 'master', args, os.environ, log_output, None)

def create_node_process(run_id, node, master_uri):
    """
    Factory for generating processes for launching local ROS
    nodes. Also registers the process with the L{ProcessMonitor} so that
    events can be generated when the process dies.
    
    @param run_id: run_id of launch
    @type  run_id: str
    @param node: node to launch. Node name must be assigned.
    @type  node: L{Node}
    @param master_uri: API URI for master node
    @type  master_uri: str
    @return: local process instance
    @rtype: L{LocalProcess}
    @raise NodeParamsException: If the node's parameters are improperly specific
    """    
    _logger.info("create_node_process: package[%s] type[%s] machine[%s] master_uri[%s]", node.package, node.type, node.machine, master_uri)
    # check input args
    machine = node.machine
    if machine is None:
        raise RLException("Internal error: no machine selected for node of type [%s/%s]"%(node.package, node.type))
    if not node.name:
        raise ValueError("node name must be assigned")

    # - setup env for process (vars must be strings for os.environ)
    env = setup_env(node, machine, master_uri)

    if not node.name:
        raise ValueError("node name must be assigned")
    
    # we have to include the counter to prevent potential name
    # collisions between the two branches

    name = "%s-%s"%(rosgraph.names.ns_join(node.namespace, node.name), _next_counter())
    if name[0] == '/':
        name = name[1:]

    _logger.info('process[%s]: env[%s]', name, env)

    args = create_local_process_args(node, machine)
    
    _logger.info('process[%s]: args[%s]', name, args)        

    # default for node.output not set is 'log'
    log_output = node.output != 'screen'
    _logger.debug('process[%s]: returning LocalProcess wrapper')
    return LocalProcess(run_id, node.package, name, args, env, log_output, \
            respawn=node.respawn, respawn_delay=node.respawn_delay, \
            required=node.required, cwd=node.cwd)


class LocalProcess(Process):
    """
    Process launched on local machine
    """
    
    def __init__(self, run_id, package, name, args, env, log_output,
            respawn=False, respawn_delay=0.0, required=False, cwd=None,
            is_node=True):
        """
        @param run_id: unique run ID for this roslaunch. Used to
          generate log directory location. run_id may be None if this
          feature is not being used.
        @type  run_id: str
        @param package: name of package process is part of
        @type  package: str
        @param name: name of process
        @type  name: str
        @param args: list of arguments to process
        @type  args: [str]
        @param env: environment dictionary for process
        @type  env: {str : str}
        @param log_output: if True, log output streams of process
        @type  log_output: bool
        @param respawn: respawn process if it dies (default is False)
        @type  respawn: bool
        @param respawn_delay: respawn process after a delay
        @type  respawn_delay: float
        @param cwd: working directory of process, or None
        @type  cwd: str
        @param is_node: (optional) if True, process is ROS node and accepts ROS node command-line arguments. Default: True
        @type  is_node: False
        """    
        super(LocalProcess, self).__init__(package, name, args, env,
                respawn, respawn_delay, required)
        self.run_id = run_id
        self.popen = None
        self.log_output = log_output
        self.started = False
        self.stopped = False
        self.cwd = cwd
        self.log_dir = None
        self.pid = -1
        self.is_node = is_node

    # NOTE: in the future, info() is going to have to be sufficient for relaunching a process
    def get_info(self):
        """
        Get all data about this process in dictionary form
        """    
        info = super(LocalProcess, self).get_info()
        info['pid'] = self.pid
        if self.run_id:
            info['run_id'] = self.run_id
        info['log_output'] = self.log_output
        if self.cwd is not None:
            info['cwd'] = self.cwd
        return info

    def _configure_logging(self):
        """
        Configure logging of node's log file and stdout/stderr
        @return: stdout log file name, stderr log file
        name. Values are None if stdout/stderr are not logged.
        @rtype: str, str
        """    
        log_dir = rospkg.get_log_dir(env=os.environ)
        if self.run_id:
            log_dir = os.path.join(log_dir, self.run_id)
        if not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError as e:
                if e.errno == 13:
                    raise RLException("unable to create directory for log file [%s].\nPlease check permissions."%log_dir)
                else:
                    raise RLException("unable to create directory for log file [%s]: %s"%(log_dir, e.strerror))
        # #973: save log dir for error messages
        self.log_dir = log_dir

        # send stdout/stderr to file. in the case of respawning, we have to
        # open in append mode
        # note: logfileerr: disabling in favor of stderr appearing in the console.
        # will likely reinstate once roserr/rosout is more properly used.
        logfileout = logfileerr = None
        logfname = self._log_name()
        
        if self.log_output:
            outf, errf = [os.path.join(log_dir, '%s-%s.log'%(logfname, n)) for n in ['stdout', 'stderr']]
            if self.respawn:
                mode = 'a'
            else:
                mode = 'w'
            logfileout = open(outf, mode)
            if is_child_mode():
                logfileerr = open(errf, mode)

        # #986: pass in logfile name to node
        node_log_file = log_dir
        if self.is_node:
            # #1595: on respawn, these keep appending
            self.args = _cleanup_remappings(self.args, '__log:=')
            self.args.append("__log:=%s"%os.path.join(log_dir, "%s.log"%(logfname)))

        return logfileout, logfileerr

    def start(self):
        """
        Start the process.
        
        @raise FatalProcessLaunch: if process cannot be started and it
        is not likely to ever succeed
        """
        super(LocalProcess, self).start()
        try:
            self.lock.acquire()
            if self.started:
                _logger.info("process[%s]: restarting os process", self.name)
            else:
                _logger.info("process[%s]: starting os process", self.name)
            self.started = self.stopped = False

            full_env = self.env

            # _configure_logging() can mutate self.args
            try:
                logfileout, logfileerr = self._configure_logging()
            except Exception as e:
                _logger.error(traceback.format_exc())
                printerrlog("[%s] ERROR: unable to configure logging [%s]"%(self.name, str(e)))
                # it's not safe to inherit from this process as
                # rostest changes stdout to a StringIO, which is not a
                # proper file.
                logfileout, logfileerr = subprocess.PIPE, subprocess.PIPE

            if self.cwd == 'node':
                cwd = os.path.dirname(self.args[0])
            elif self.cwd == 'cwd':
                cwd = os.getcwd()
            elif self.cwd == 'ros-root':
                cwd = get_ros_root()
            else:
                cwd = rospkg.get_ros_home()
            if not os.path.exists(cwd):
                try:
                    os.makedirs(cwd)
                except OSError:
                    # exist_ok=True
                    pass

            _logger.info("process[%s]: start w/ args [%s]", self.name, self.args)
            _logger.info("process[%s]: cwd will be [%s]", self.name, cwd)

            try:
                self.popen = subprocess.Popen(self.args, cwd=cwd, stdout=logfileout, stderr=logfileerr, env=full_env, close_fds=True, preexec_fn=os.setsid)
            except OSError as e:
                self.started = True # must set so is_alive state is correct
                _logger.error("OSError(%d, %s)", e.errno, e.strerror)
                if e.errno == 8: #Exec format error
                    raise FatalProcessLaunch("Unable to launch [%s]. \nIf it is a script, you may be missing a '#!' declaration at the top."%self.name)
                elif e.errno == 2: #no such file or directory
                    raise FatalProcessLaunch("""Roslaunch got a '%s' error while attempting to run:

%s

Please make sure that all the executables in this command exist and have
executable permission. This is often caused by a bad launch-prefix."""%(e.strerror, ' '.join(self.args)))
                else:
                    raise FatalProcessLaunch("unable to launch [%s]: %s"%(' '.join(self.args), e.strerror))
                
            self.started = True
            # Check that the process is either still running (poll returns
            # None) or that it completed successfully since when we
            # launched it above (poll returns the return code, 0).
            poll_result = self.popen.poll()
            if poll_result is None or poll_result == 0:
                self.pid = self.popen.pid
                printlog_bold("process[%s]: started with pid [%s]"%(self.name, self.pid))
                return True
            else:
                printerrlog("failed to start local process: %s"%(' '.join(self.args)))
                return False
        finally:
            self.lock.release()

    def _log_name(self):
        return self.name.replace('/', '-')
    
    def is_alive(self):
        """
        @return: True if process is still running
        @rtype: bool
        """
        if not self.started: #not started yet
            return True
        if self.stopped or self.popen is None:
            if self.time_of_death is None:
                self.time_of_death = time.time()
            return False
        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            if self.time_of_death is None:
                self.time_of_death = time.time()
            return False
        return True

    def get_exit_description(self):
        """
        @return: human-readable description of exit state 
        @rtype: str
        """
        if self.exit_code is None:
            output = 'process has died without exit code [pid %s, cmd %s].'%(self.pid, ' '.join(self.args))
        elif self.exit_code != 0:
            output = 'process has died [pid %s, exit code %s, cmd %s].'%(self.pid, self.exit_code, ' '.join(self.args))
        else:
            output = 'process has finished cleanly'
                
        if self.log_dir:
            # #973: include location of output location in message
            output += '\nlog file: %s*.log'%(os.path.join(self.log_dir, self._log_name()))
        return output

    def _stop_unix(self, errors):
        """
        UNIX implementation of process killing

        @param errors: error messages. stop() will record messages into this list.
        @type  errors: [str]
        """
        self.exit_code = self.popen.poll() 
        if self.exit_code is not None:
            _logger.debug("process[%s].stop(): process has already returned %s", self.name, self.exit_code)
            #print "process[%s].stop(): process has already returned %s"%(self.name, self.exit_code)                
            self.popen = None
            self.stopped = True
            return

        pid = self.popen.pid
        pgid = os.getpgid(pid)
        _logger.info("process[%s]: killing os process with pid[%s] pgid[%s]", self.name, pid, pgid)

        try:
            # Start with SIGINT and escalate from there.
            _logger.info("[%s] sending SIGINT to pgid [%s]", self.name, pgid)                                    
            os.killpg(pgid, signal.SIGINT)
            _logger.info("[%s] sent SIGINT to pgid [%s]", self.name, pgid)
            timeout_t = time.time() + _TIMEOUT_SIGINT
            retcode = self.popen.poll()                
            while time.time() < timeout_t and retcode is None:
                time.sleep(0.1)
                retcode = self.popen.poll()
            # Escalate non-responsive process
            if retcode is None:
                printerrlog("[%s] escalating to SIGTERM"%self.name)
                timeout_t = time.time() + _TIMEOUT_SIGTERM
                os.killpg(pgid, signal.SIGTERM)                
                _logger.info("[%s] sent SIGTERM to pgid [%s]"%(self.name, pgid))
                retcode = self.popen.poll()
                while time.time() < timeout_t and retcode is None:
                    time.sleep(0.2)
                    _logger.debug('poll for retcode')
                    retcode = self.popen.poll()
                if retcode is None:
                    printerrlog("[%s] escalating to SIGKILL"%self.name)
                    errors.append("process[%s, pid %s]: required SIGKILL. May still be running."%(self.name, pid))
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        _logger.info("[%s] sent SIGKILL to pgid [%s]"%(self.name, pgid))
                        # #2096: don't block on SIGKILL, because this results in more orphaned processes overall
                        #self.popen.wait()
                        #os.wait()
                        _logger.info("process[%s]: sent SIGKILL", self.name)
                    except OSError as e:
                        if e.args[0] == 3:
                            printerrlog("no [%s] process with pid [%s]"%(self.name, pid))
                        else:
                            printerrlog("errors shutting down [%s], see log for details"%self.name)
                            _logger.error(traceback.format_exc())
                else:
                    _logger.info("process[%s]: SIGTERM killed with return value %s", self.name, retcode)
            else:
                _logger.info("process[%s]: SIGINT killed with return value %s", self.name, retcode)
                
        finally:
            self.popen = None

    def _stop_win32(self, errors):
        """
        Win32 implementation of process killing. In part, refer to

          http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/347462
        
        Note that it doesn't work as completely as _stop_unix as it can't utilise
        group id's. This means that any program which forks children underneath it
        won't get caught by this kill mechanism.
        
        @param errors: error messages. stop() will record messages into this list.
        @type  errors: [str]
        """
        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            _logger.debug("process[%s].stop(): process has already returned %s", self.name, self.exit_code)
            self.popen = None
            self.stopped = True
            return

        pid = self.popen.pid
        _logger.info("process[%s]: killing os process/subprocesses with pid[%s]", self.name, pid)
        # windows has no group id's :(
        try:
            # Start with SIGINT and escalate from there.
            _logger.info("[%s] sending SIGINT to pgid [%s]", self.name, pid)
            os.kill(pid, signal.SIGINT)
            _logger.info("[%s] sent SIGINT to pgid [%s]", self.name, pid)
            timeout_t = time.time() + _TIMEOUT_SIGINT
            retcode = self.popen.poll()
            while time.time() < timeout_t and retcode is None:
                time.sleep(0.1)
                retcode = self.popen.poll()
            # Escalate non-responsive process
            if retcode is None:
                printerrlog("[%s] escalating to SIGTERM"%self.name)
                timeout_t = time.time() + _TIMEOUT_SIGTERM
                os.killpg(pid, signal.SIGTERM)
                _logger.info("[%s] sent SIGTERM to pid [%s]"%(self.name, pid))
                retcode = self.popen.poll()
                while time.time() < timeout_t and retcode is None:
                    time.sleep(0.2)
                    _logger.debug('poll for retcode')
                    retcode = self.popen.poll()
                if retcode is None:
                    printerrlog("[%s] escalating to SIGKILL"%self.name)
                    errors.append("process[%s, pid %s]: required SIGKILL. May still be running."%(self.name, pid))
                    try:
                        os.killpg(pid, signal.SIGKILL)
                        _logger.info("[%s] sent SIGKILL to pid [%s]"%(self.name, pid))
                        # #2096: don't block on SIGKILL, because this results in more orphaned processes overall
                        #self.popen.wait()
                        #os.wait()
                        _logger.info("process[%s]: sent SIGKILL", self.name)
                    except OSError as e:
                        if e.args[0] == 3:
                            printerrlog("no [%s] process with pid [%s]"%(self.name, pid))
                        else:
                            printerrlog("errors shutting down [%s], see log for details"%self.name)
                            _logger.error(traceback.format_exc())
                else:
                    _logger.info("process[%s]: SIGTERM killed with return value %s", self.name, retcode)
            else:
                _logger.info("process[%s]: SIGINT killed with return value %s", self.name, retcode)
        finally:
            self.popen = None
			
    def stop(self, errors=None):
        """
        Stop the process. Record any significant error messages in the errors parameter
        
        @param errors: error messages. stop() will record messages into this list.
        @type  errors: [str]
        """
        if errors is None:
            errors = []
        super(LocalProcess, self).stop(errors)
        self.lock.acquire()        
        try:
            try:
                _logger.debug("process[%s].stop() starting", self.name)
                if self.popen is None:
                    _logger.debug("process[%s].stop(): popen is None, nothing to kill") 
                    return
                if sys.platform in ['win32']: # cygwin seems to be ok
                    self._stop_win32(errors)
                else:
                    self._stop_unix(errors)
            except:
                #traceback.print_exc() 
                _logger.error("[%s] EXCEPTION %s", self.name, traceback.format_exc())                                
        finally:
            self.stopped = True
            self.lock.release()


# #1595
def _cleanup_remappings(args, prefix):
    """
    Remove all instances of args that start with prefix. This is used
    to remove args that were previously added (and are now being
    regenerated due to respawning)
    """    
    existing_args = [a for a in args if a.startswith(prefix)]
    for a in existing_args:
        args.remove(a)
    return args
