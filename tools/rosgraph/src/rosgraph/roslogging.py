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

"""
Library for configuring python logging to standard ROS locations (e.g. ROS_LOG_DIR).
"""

import os
import sys
import time
import logging
import logging.config
import inspect

import yaml

import rospkg
from rospkg.environment import ROS_LOG_DIR

class LoggingException(Exception): pass

class RospyLogger(logging.getLoggerClass()):
    def findCaller(self, stack_info=False):
        """
        Find the stack frame of the caller so that we can note the source
        file name, line number, and function name with class name if possible.
        """
        file_name, lineno, func_name = super(RospyLogger, self).findCaller()[:3]
        file_name = os.path.normcase(file_name)

        f = inspect.currentframe()
        if f is not None:
            f = f.f_back
        while hasattr(f, "f_code"):
            # Search for the right frame using the data already found by parent class.
            co = f.f_code
            filename = os.path.normcase(co.co_filename)
            if filename == file_name and f.f_lineno == lineno and co.co_name == func_name:
                break
            f = f.f_back

        # Jump up two more frames, as the logger methods have been double wrapped.
        if f.f_back and f.f_code and f.f_code.co_name == '_base_logger':
            f = f.f_back
            if f.f_back:
                f = f.f_back
        co = f.f_code
        func_name = co.co_name

        # Now extend the function name with class name, if available.
        try:
            class_name = f.f_locals['self'].__class__.__name__
            func_name = '%s.%s' % (class_name, func_name)
        except KeyError:  # if the function is unbound, there is no self.
            pass

        if sys.version_info > (3, 2):
            # Dummy last argument to match Python3 return type
            return co.co_filename, f.f_lineno, func_name, None
        else:
            return co.co_filename, f.f_lineno, func_name

logging.setLoggerClass(RospyLogger)

def renew_latest_logdir(logfile_dir):
    log_dir = os.path.dirname(logfile_dir)
    latest_dir = os.path.join(log_dir, 'latest')
    if os.path.lexists(latest_dir):
        if not os.path.islink(latest_dir):
            return False
        os.remove(latest_dir)
    os.symlink(logfile_dir, latest_dir)
    return True

def configure_logging(logname, level=logging.INFO, filename=None, env=None):
    """
    Configure Python logging package to send log files to ROS-specific log directory
    :param logname str: name of logger, ``str``
    :param filename: filename to log to. If not set, a log filename
        will be generated using logname, ``str``
    :param env: override os.environ dictionary, ``dict``
    :returns: log file name, ``str``
    :raises: :exc:`LoggingException` If logging cannot be configured as specified
    """
    if env is None:
        env = os.environ

    logname = logname or 'unknown'
    log_dir = rospkg.get_log_dir(env=env)
    
    # if filename is not explicitly provided, generate one using logname
    if not filename:
        log_filename = os.path.join(log_dir, '%s-%s.log'%(logname, os.getpid()))
    else:
        log_filename = os.path.join(log_dir, filename)

    logfile_dir = os.path.dirname(log_filename)
    if not os.path.exists(logfile_dir):
        try:
            makedirs_with_parent_perms(logfile_dir)
        except OSError:
            # cannot print to screen because command-line tools with output use this
            if os.path.exists(logfile_dir):
                # We successfully created the logging folder, but could not change
                # permissions of the new folder to the same as the parent folder
                sys.stderr.write("WARNING: Could not change permissions for folder [%s], make sure that the parent folder has correct permissions.\n"%logfile_dir)
            else:
                # Could not create folder
                sys.stderr.write("WARNING: cannot create log directory [%s]. Please set %s to a writable location.\n"%(logfile_dir, ROS_LOG_DIR))
            return None
    elif os.path.isfile(logfile_dir):
        raise LoggingException("Cannot save log files: file [%s] is in the way"%logfile_dir)

    # the log dir itself should not be symlinked as latest
    if logfile_dir != log_dir:
        if sys.platform not in ['win32']:
            try:
                success = renew_latest_logdir(logfile_dir)
                if not success:
                    sys.stderr.write("INFO: cannot create a symlink to latest log directory\n")
            except OSError as e:
                sys.stderr.write("INFO: cannot create a symlink to latest log directory: %s\n" % e)

    config_file = os.environ.get('ROS_PYTHON_LOG_CONFIG_FILE')
    if not config_file:
        # search for logging config file in ROS_HOME, ROS_ETC_DIR or relative
        # to the rosgraph package directory.
        rosgraph_d = rospkg.RosPack().get_path('rosgraph')
        for config_dir in [os.path.join(rospkg.get_ros_home(), 'config'),
                           rospkg.get_etc_ros_dir(),
                           os.path.join(rosgraph_d, 'conf')]:
            for extension in ('conf', 'yaml'):
                f = os.path.join(config_dir,
                                 'python_logging{}{}'.format(os.path.extsep,
                                                             extension))
                if os.path.isfile(f):
                    config_file = f
                    break
            if config_file is not None:
                break

    if config_file is None or not os.path.isfile(config_file):
        # logging is considered soft-fail
        sys.stderr.write("WARNING: cannot load logging configuration file, logging is disabled\n")
        logging.getLogger(logname).setLevel(logging.CRITICAL)
        return log_filename
    
    # pass in log_filename as argument to pylogging.conf
    os.environ['ROS_LOG_FILENAME'] = log_filename
    if config_file.endswith(('.yaml', '.yml')):
        with open(config_file) as f:
            dict_conf = yaml.safe_load(f)
        dict_conf.setdefault('version', 1)
        logging.config.dictConfig(dict_conf)
    else:
        # #3625: disabling_existing_loggers=False
        logging.config.fileConfig(config_file, disable_existing_loggers=False)

    return log_filename


def makedirs_with_parent_perms(p):
    """
    Create the directory using the permissions of the nearest
    (existing) parent directory. This is useful for logging, where a
    root process sometimes has to log in the user's space.
    :param p: directory to create, ``str``
    """    
    p = os.path.abspath(p)
    parent = os.path.dirname(p)
    # recurse upwards, checking to make sure we haven't reached the
    # top
    if not os.path.exists(p) and p and parent != p:
        makedirs_with_parent_perms(parent)
        s = os.stat(parent)
        os.mkdir(p)

        # if perms of new dir don't match, set anew
        s2 = os.stat(p)
        if s.st_uid != s2.st_uid or s.st_gid != s2.st_gid:
            os.chown(p, s.st_uid, s.st_gid)
        if s.st_mode != s2.st_mode:
            os.chmod(p, s.st_mode)    

_logging_to_rospy_names = {
    'DEBUG': ('DEBUG', '\033[32m'),
    'INFO': ('INFO', None),
    'WARNING': ('WARN', '\033[33m'),
    'ERROR': ('ERROR', '\033[31m'),
    'CRITICAL': ('FATAL', '\033[31m')
}
_color_reset = '\033[0m'
_defaultFormatter = logging.Formatter()

class RosStreamHandler(logging.Handler):
    def __init__(self, colorize=True, stdout=None, stderr=None):
        super(RosStreamHandler, self).__init__()
        self._stdout = stdout or sys.stdout
        self._stderr = stderr or sys.stderr
        self._colorize = colorize
        try:
            from rospy.rostime import get_time, is_wallclock
            self._get_time = get_time
            self._is_wallclock = is_wallclock
        except ImportError:
            self._get_time = None
            self._is_wallclock = None

    def emit(self, record):
        level, color = _logging_to_rospy_names[record.levelname]
        record_message = _defaultFormatter.format(record)
        msg = os.environ.get(
            'ROSCONSOLE_FORMAT', '[${severity}] [${time}]: ${message}')
        msg = msg.replace('${severity}', level)
        msg = msg.replace('${message}', str(record_message))
        msg = msg.replace('${walltime}', '%f' % time.time())
        msg = msg.replace('${thread}', str(record.thread))
        msg = msg.replace('${logger}', str(record.name))
        msg = msg.replace('${file}', str(record.pathname))
        msg = msg.replace('${line}', str(record.lineno))
        msg = msg.replace('${function}', str(record.funcName))
        try:
            from rospy import get_name
            node_name = get_name()
        except ImportError:
            node_name = '<unknown_node_name>'
        msg = msg.replace('${node}', node_name)
        time_str = '%f' % time.time()
        if self._get_time is not None and not self._is_wallclock():
            time_str += ', %f' % self._get_time()
        msg = msg.replace('${time}', time_str)
        msg += '\n'
        if record.levelno < logging.WARNING:
            self._write(self._stdout, msg, color)
        else:
            self._write(self._stderr, msg, color)

    def _write(self, fd, msg, color):
        if self._colorize and color and hasattr(fd, 'isatty') and fd.isatty():
            msg = color + msg + _color_reset
        fd.write(msg)
