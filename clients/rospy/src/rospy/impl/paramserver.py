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

"""Parameter Server Cache"""


import threading
from rosgraph.names import GLOBALNS, SEP

class ParamServerCache(object):
    """
    Cache of values on the parameter server. Implementation
    is just a thread-safe dictionary.
    """
    
    def __init__(self):
        self.lock = threading.Lock()
        self.d = None
        self.notifier = None
        
    ## Delete parameter from cache
    def delete(self, key):
        with self.lock:
            # partially borrowed from rosmaster/paramserver.py
            if key == GLOBALNS:
                raise KeyError("cannot delete root of parameter tree")
            elif self.d is None:
                raise KeyError(key)
            else:
                # key is global, so first split is empty
                namespaces = [x for x in key.split(SEP) if x]
                # - last namespace is the actual key we're deleting
                value_key = namespaces[-1]
                namespaces = namespaces[:-1]
                d = self.d
                # - descend tree to the node we're setting
                for ns in namespaces:
                    if type(d) != dict or ns not in d:
                        raise KeyError(key)
                    else:
                        d = d[ns]

                if value_key not in d:
                    raise KeyError(key)
                else:
                    del d[value_key]

    def set_notifier(self, notifier):
        """
        Notifier implements any parameter subscription logic. The
        notifier should be a function that takes in a key and value
        that represents a parameter update. Notifier is called under
        lock and thus must not implement any lengthy computation.
        """
        self.notifier = notifier
        
    def update(self, key, value):
        """
        Update the value of the parameter in the cache
        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @type  value: str
        @raise: KeyError if key is not already in the cache.
        """
        with self.lock:
            # partially borrowed from rosmaster/paramserver.py
            namespaces = [x for x in key.split(SEP) if x]
            # - last namespace is the actual key we're storing in
            value_key = namespaces[-1]
            namespaces = namespaces[:-1]
            d = self.d
            if d is None:
                raise KeyError(key)
            # - descend tree to the node we're setting
            for ns in namespaces:
                if ns not in d:
                    raise KeyError(key)
                else:
                    d = d[ns]

            if value_key not in d:
                raise KeyError(key)

            d[value_key] = value
            if self.notifier is not None:
                self.notifier(key, value)
                
    def set(self, key, value):
        """
        Set the value of the parameter in the cache. This is a
        prerequisite of calling update().
        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @type  value: str
        """
        with self.lock:
            # partially borrowed from rosmaster/paramserver.py
            if key == GLOBALNS:
                if type(value) != dict:
                    raise TypeError("cannot set root of parameter tree to "
                                    "non-dictionary")
                self.d = value
            else:
                namespaces = [x for x in key.split(SEP) if x]
                # - last namespace is the actual key we're storing in
                value_key = namespaces[-1]
                namespaces = namespaces[:-1]
                if self.d is None:
                    self.d = {}
                d = self.d
                # - descend tree to the node we're setting
                for ns in namespaces:
                    if ns not in d:
                        new_d = {}
                        d[ns] = new_d
                        d = new_d
                    else:
                        val = d[ns]
                        # implicit type conversion of value to namespace
                        if type(val) != dict:
                            d[ns] = val = {}
                        d = val

                d[value_key] = value

    def get(self, key):
        """
        @param key: parameter key
        @type  key: str
        @return: Current value for parameter
        @raise: KeyError
        """
        with self.lock:
            # borrowed from rosmaster/paramserver.py
            if self.d is None:
                raise KeyError(key)
            val = self.d
            if key != GLOBALNS:
                # split by the namespace separator, ignoring empty splits
                namespaces = [x for x in key.split(SEP) if x]
                for ns in namespaces:
                    if not type(val) == dict:
                        raise KeyError(val)
                    val = val[ns]
            return val

_param_server_cache = None
def get_param_server_cache():
    """
    Get a handle on the client-wide parameter server cache
    """
    global _param_server_cache
    if _param_server_cache is None:
        _param_server_cache = ParamServerCache()        
    return _param_server_cache
