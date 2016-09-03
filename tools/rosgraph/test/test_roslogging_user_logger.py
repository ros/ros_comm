# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Kentaro Wada.
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
Test for rosgraph.roslogger with custom logger defined by user,
to ensure the custom logger won't be overwritten by RospyLogger defined
in rosgraph.roslogger.
"""

import logging
import os
from StringIO import StringIO
import sys

from nose.tools import assert_regexp_matches


# set user defined custom logger
class UserCustomLogger(logging.Logger):
    def findCaller(self):
        """Returns static caller.

        This method is being overwritten in rosgraph.roslogging.
        """
        return '<filename>', '<lineno>', '<func_name>'

    def _log(self, level, msg, args, exc_info=None, extra=None):
        """Write log with ROS_IP.

        This method is not being overwritten in rosgraph.roslogging.
        """
        ros_ip = os.environ.get('ROS_IP', '<unknown ros_ip>')
        msg = '%s %s' % (ros_ip, msg)
        logging.Logger._log(self, level, msg, args, exc_info, extra)

logging.setLoggerClass(UserCustomLogger)


import rosgraph.roslogging


os.environ['ROS_IP'] = '127.0.0.1'
os.environ['ROSCONSOLE_FORMAT'] = ' '.join([
    '${severity}',
    '${message}',
    '${walltime}',
    '${thread}',
    '${logger}',
    '${file}',
    '${line}',
    '${function}',
    '${node}',
    '${time}',
])
rosgraph.roslogging.configure_logging('test_rosgraph', logging.INFO)
loginfo = logging.getLogger('rosout').info

# Remap stdout for testing
f = StringIO()
sys.stdout = f


loginfo('on module')


def logging_on_function():
    loginfo('on function')

logging_on_function()


class LoggingOnClass(object):

    def __init__(self):
        loginfo('on method')

LoggingOnClass()


def test_roslogging_user_logger():
    this_file = os.path.abspath(__file__)
    # this is necessary to avoid test fails because of .pyc cache file
    base, ext = os.path.splitext(this_file)
    if ext == '.pyc':
        this_file = base + '.py'

    for i, loc in enumerate(['module', 'function', 'method']):
        log_out = ' '.join([
            'INFO',
            os.environ['ROS_IP'],
            'on ' + loc,
            '[0-9]*\.[0-9]*',
            '[0-9]*',
            'rosout',
            '.*',
            '[0-9]*',
            '.*',
            '/unnamed',
            '[0-9]*\.[0-9]*',
        ])
        assert_regexp_matches(f.getvalue().splitlines()[i], log_out)


sys.stdout = sys.__stdout__
