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
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import sys

from nose.tools import assert_regexp_matches

import rosgraph.roslogging


# set user defined custom logger
class UserCustomLogger(logging.Logger):
    def findCaller(self, stack_info=False, stacklevel=1):
        """Returns static caller.

        This method is being overwritten in rosgraph.roslogging.
        """
        if sys.version_info > (3, 2):
            # Dummy last argument to match Python3 return type
            return '<filename>', '<lineno>', '<func_name>', None
        else:
            return '<filename>', '<lineno>', '<func_name>'

    def _log(self, level, msg, args, exc_info=None, extra=None):
        """Write log with ROS_IP.

        This method is not being overwritten in rosgraph.roslogging.
        """
        ros_ip = os.environ.get('ROS_IP', '<unknown ros_ip>')
        msg = '%s %s' % (ros_ip, msg)
        logging.Logger._log(self, level, msg, args, exc_info, extra)


def setup_module():
    logging.setLoggerClass(UserCustomLogger)


def teardown_module():
    logging.setLoggerClass(rosgraph.roslogging.RospyLogger)


def test_roslogging_user_logger():
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ['ROSCONSOLE_FORMAT'] = ' '.join([
        '${severity}',
        '${message}',
        '${walltime}',
        '${walltime:%Y-%m-%d %H:%M:%S}',
        '${thread}',
        '${logger}',
        '${file}',
        '${line}',
        '${function}',
        '${node}',
        '${time}',
        '${time:%Y-%m-%d %H:%M:%S}',
    ])
    rosgraph.roslogging.configure_logging('test_rosgraph', logging.INFO)
    loginfo = logging.getLogger('rosout.custom_logger_test').info

    # Remap stdout for testing
    try:
        from cStringIO import StringIO
    except ImportError:
        from io import StringIO
    lout = StringIO()
    lerr = StringIO()
    test_ros_handler = rosgraph.roslogging.RosStreamHandler(colorize=False, stdout=lout, stderr=lerr)

    custom_logger = logging.getLogger('rosout.custom_logger_test')
    assert len(custom_logger.handlers) == 0

    # log messages will be propagated to parent
    rosout_logger = logging.getLogger('rosout')
    assert isinstance(rosout_logger.handlers[0], rosgraph.roslogging.RosStreamHandler)
    default_ros_handler = rosout_logger.handlers[0]

    # hack to replace the stream handler with a debug version
    rosout_logger.removeHandler(default_ros_handler)
    rosout_logger.addHandler(test_ros_handler)

    try:
        # Logging
        msg = 'Hello world.'
        loginfo(msg)

        log_expected = ' '.join([
            'INFO',
            os.environ['ROS_IP'],
            msg,
            r'[0-9]*\.[0-9]*',
            r'[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}',
            '[0-9]*',
            'rosout.custom_logger_test',
            '<filename>',
            '<lineno>',
            '<func_name>',
            # depending if rospy.get_name() is available
            '(/unnamed|<unknown_node_name>)',
            r'[0-9]*\.[0-9]*',
            r'[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}',
        ])
        assert_regexp_matches(lout.getvalue().strip(), log_expected)

    finally:
        # restoring default ros handler
        logging.getLogger('rosout').removeHandler(test_ros_handler)
        logging.getLogger('rosout').addHandler(default_ros_handler)
        lout.close()
        lerr.close()


