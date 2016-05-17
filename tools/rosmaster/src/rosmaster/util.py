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
Utility routines for rosmaster.
"""

try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from defusedxml.xmlrpc import monkey_patch
monkey_patch()
del monkey_patch

import rosgraph.security as security

_proxies = {} #cache ServerProxys
def xmlrpcapi(uri, node_name):
    """
    @return: instance for calling remote server or None if not a valid URI
    @rtype: xmlrpc.client.ServerProxy
    """
    if uri is None:
        return None
    uriValidate = urlparse(uri)
    if not uriValidate[0] or not uriValidate[1]:
        return None
    if not uri in _proxies:
        _proxies[uri] = security.get().xmlrpcapi(uri, node_name)
    return _proxies[uri]


def remove_server_proxy(uri):
    if uri in _proxies:
        del _proxies[uri]
