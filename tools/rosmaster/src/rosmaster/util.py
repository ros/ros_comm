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

import collections

try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse
try:
    from urllib.parse import splittype
except ImportError:
    from urllib import splittype
import threading

from rosgraph.xmlrpc import ServerProxy

from defusedxml.xmlrpc import monkey_patch
monkey_patch()
del monkey_patch

_proxies = {} #cache ServerProxys

_global_lock = threading.Lock()  # global lock for the _uri_locks dict
_uri_locks = collections.defaultdict(threading.Lock)  # one lock object per uri
_proxies = {}  #cache ServerProxys

def _get_lock(uri):
    with _global_lock:
        return _uri_locks[uri]


def xmlrpcapi(uri):
    """
    @return: instance for calling remote server or None if not a valid URI
    @rtype: rosgraph.xmlrpc.ServerProxy
    """
    if uri is None:
        return None
    uriValidate = urlparse(uri)
    if not uriValidate[0] or not uriValidate[1]:
        return None
    with _get_lock(uri):
        if uri not in _proxies:
            _proxies[uri] = ServerProxy(uri)
        return _proxies[uri]


def remove_server_proxy(uri):
    with _global_lock:
        with _uri_locks[uri]:
            if uri in _proxies:
                del _proxies[uri]
            del _uri_locks[uri]
