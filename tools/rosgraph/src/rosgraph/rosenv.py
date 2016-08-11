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

import os
import sys
import sros_consts as sros_consts

ROS_MASTER_URI   = "ROS_MASTER_URI"
SROS_KEYSERVER_URI   = "SROS_KEYSERVER_URI"

ROS_IP           ="ROS_IP"
ROS_IPV6         ="ROS_IPV6"
ROS_HOSTNAME     ="ROS_HOSTNAME"
ROS_NAMESPACE    ="ROS_NAMESPACE"

def get_master_uri(env=None, argv=None):
    """
    Get the :envvar:`ROS_MASTER_URI` setting from the command-line args or
    environment, command-line args takes precedence.

    :param env: override environment dictionary, ``dict``
    :param argv: override ``sys.argv``, ``[str]``
    :raises: :exc:`ValueError` If :envvar:`ROS_MASTER_URI` value is invalidly
      specified 
    """    
    if env is None:
        env = os.environ
    if argv is None:
        argv = sys.argv
    try:
        for arg in argv:
            if arg.startswith('__master:='):
                val = None
                try:
                    _, val = arg.split(':=')
                except:
                    pass
                
                # we ignore required here because there really is no
                # correct return value as the configuration is bad
                # rather than unspecified
                if not val:
                    raise ValueError("__master remapping argument '%s' improperly specified"%arg)
                return val
        if 'SROS_SECURITY' in env and \
            env['SROS_SECURITY'] == sros_consts.SecuityModes.TLSv1_2 and \
            env[ROS_MASTER_URI].split('://')[0] == 'http':
            # SROS_SECURITY is set to SSL but ROS_MASTER_URI is not HTTPS. I'll fix that for you
            return 'https://' + env[ROS_MASTER_URI].split('://')[1]
            
        return env[ROS_MASTER_URI]
    except KeyError as e:
        return None

def parse_uri(uri):
    protocol, address_port = uri.split('://')
    address, port = address_port.rstrip('/').split(':')
    return protocol, address, int(port)

def get_keyserver_uri(env=None, argv=None):
    """
    Get the :envvar:`SROS_KEYSERVER_URI` setting from the command-line args or
    environment, command-line args takes precedence.

    :param env: override environment dictionary, ``dict``
    :param argv: override ``sys.argv``, ``[str]``
    :raises: :exc:`ValueError` If :envvar:both `ROS_MASTER_URI`
      and `SROS_KEYSERVER_URI` values are invalidly specified 
    """
    if env is None:
        env = os.environ
    if argv is None:
        argv = sys.argv
    try:
        for arg in argv:
            if arg.startswith('__keyserver:='):
                val = None
                try:
                    _, val = arg.split(':=')
                except:
                    pass

                # we ignore required here because there really is no
                # correct return value as the configuration is bad
                # rather than unspecified
                if not val:
                    raise ValueError("__keyserver remapping argument '%s' improperly specified" % arg)
                return val
        if SROS_KEYSERVER_URI in env:
            return env[SROS_KEYSERVER_URI]
        else:
            #defult to using ROS_MASTER_URI to get address and port
            master_uri = get_master_uri(env, argv)
            protocol, address, port = parse_uri(master_uri)
            keyserver_uri = '%s://%s:%d' % (protocol, address, port - 1)
            return keyserver_uri
    except KeyError as e:
        return None