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
# Revision $Id: xmlrpc.py 15336 2011-11-07 20:43:00Z kwc $

from __future__ import print_function

"""
Common XML-RPC for higher-level libraries running XML-RPC libraries in
ROS. In particular, this library provides common handling for URI
calculation based on ROS environment variables.

The common entry point for most libraries is the L{XmlRpcNode} class.
"""

import logging
import select
import socket

try:
    import _thread
except ImportError:
    import thread as _thread

import traceback

try:
    from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler, resolve_dotted_attribute #Python 3.x
except ImportError:
    from SimpleXMLRPCServer import SimpleXMLRPCServer #Python 2.x
    from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler #Python 2.x
    from SimpleXMLRPCServer import resolve_dotted_attribute #Python 2.x

try:
    import socketserver
except ImportError:
    import SocketServer as socketserver

import rosgraph.network
import rosgraph.security as security


import xmlrpclib
from xmlrpclib import Fault
import sys

def isstring(s):
    """Small helper version to check an object is a string in a way that works
    for both Python 2 and 3
    """
    try:
        return isinstance(s, basestring)
    except NameError:
        return isinstance(s, str)

class SilenceableXMLRPCRequestHandler(SimpleXMLRPCRequestHandler):
    def log_message(self, format, *args):
        if 0:
            SimpleXMLRPCRequestHandler.log_message(self, format, *args)

    def handle_one_request(self):
        """
        Overrides SimpleXMLRPCDispatcher to pass connection's context to _dispatch
        
        Handle a single HTTP request.

        You normally don't need to override this method; see the class
        __doc__ string for information on how to handle specific HTTP
        commands such as GET and POST.

        """
        try:
            try:
                self.peercert = self.connection.getpeercert(binary_form=True)
            except:
                self.peercert = None
            self.raw_requestline = self.rfile.readline(65537)
            if len(self.raw_requestline) > 65536:
                self.requestline = ''
                self.request_version = ''
                self.command = ''
                self.send_error(414)
                return
            if not self.raw_requestline:
                self.close_connection = 1
                return
            if not self.parse_request():
                # An error code has been sent, just exit
                return
            mname = 'do_' + self.command
            if not hasattr(self, mname):
                self.send_error(501, "Unsupported method (%r)" % self.command)
                return
            method = getattr(self, mname)
            method()
            self.wfile.flush() #actually send the response if not already done.
        except socket.timeout, e:
            #a read or a write timed out.  Discard this connection
            self.log_error("Request timed out: %r", e)
            self.close_connection = 1
            return

    def do_POST(self):
        """
        Overrides SimpleXMLRPCRequestHandler to pass connection's peercert as context to _marshaled_dispatch
        
        Handles the HTTP POST request.

        Attempts to interpret all HTTP POST requests as XML-RPC calls,
        which are forwarded to the server's _dispatch method for handling.
        """

        # Check that the path is legal
        if not self.is_rpc_path_valid():
            self.report_404()
            return

        try:
            # Get arguments by reading body of request.
            # We read this in chunks to avoid straining
            # socket.read(); around the 10 or 15Mb mark, some platforms
            # begin to have problems (bug #792570).
            max_chunk_size = 10 * 1024 * 1024
            size_remaining = int(self.headers["content-length"])
            L = []
            while size_remaining:
                chunk_size = min(size_remaining, max_chunk_size)
                chunk = self.rfile.read(chunk_size)
                if not chunk:
                    break
                L.append(chunk)
                size_remaining -= len(L[-1])
            data = ''.join(L)

            data = self.decode_request_content(data)

            if data is None:
                return  # response has been sent

            # In previous versions of SimpleXMLRPCServer, _dispatch
            # could be overridden in this class, instead of in
            # SimpleXMLRPCDispatcher. To maintain backwards compatibility,
            # check to see if a subclass implements _dispatch and dispatch
            # using that method if present.
            response = self.server._marshaled_dispatch(
                data, getattr(self, '_dispatch', None), self.path, context=self.peercert
            )
        except Exception, e:  # This should only happen if the module is buggy
            # internal error, report as HTTP server error
            self.send_response(500)

            # Send information about the exception if requested
            if hasattr(self.server, '_send_traceback_header') and \
                    self.server._send_traceback_header:
                self.send_header("X-exception", str(e))
                self.send_header("X-traceback", traceback.format_exc())

            self.send_header("Content-length", "0")
            self.end_headers()
        else:
            # got a valid XML RPC response
            self.send_response(200)
            self.send_header("Content-type", "text/xml")
            if self.encode_threshold is not None:
                if len(response) > self.encode_threshold:
                    q = self.accept_encodings().get("gzip", 0)
                    if q:
                        try:
                            response = xmlrpclib.gzip_encode(response)
                            self.send_header("Content-Encoding", "gzip")
                        except NotImplementedError:
                            pass
            self.send_header("Content-length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)
    
class ThreadingXMLRPCServer(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    """
    Adds ThreadingMixin to SimpleXMLRPCServer to support multiple concurrent
    requests via threading. Also makes logging toggleable.
    """
    def __init__(self, addr, log_requests=1):
        """
        Overrides SimpleXMLRPCServer to set option to allow_reuse_address.
        """
        # allow_reuse_address defaults to False in Python 2.4.  We set it 
        # to True to allow quick restart on the same port.  This is equivalent 
        # to calling setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
        self.allow_reuse_address = True
        if rosgraph.network.use_ipv6():
            logger = logging.getLogger('xmlrpc')
            # The XMLRPC library does not support IPv6 out of the box
            # We have to monipulate private members and duplicate
            # code from the constructor.
            # TODO IPV6: Get this into SimpleXMLRPCServer 
            SimpleXMLRPCServer.__init__(self, addr, SilenceableXMLRPCRequestHandler, log_requests,  bind_and_activate=False)
            self.address_family = socket.AF_INET6
            self.socket = socket.socket(self.address_family, self.socket_type)
            logger.info('binding ipv6 xmlrpc socket to' + str(addr))
            # TODO: set IPV6_V6ONLY to 0:
            # self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            self.server_bind()
            self.server_activate()
            logger.info('bound to ' + str(self.socket.getsockname()[0:2]))
        else:
            SimpleXMLRPCServer.__init__(self, addr, SilenceableXMLRPCRequestHandler, log_requests)

    def handle_error(self, request, client_address):
        """
        override ThreadingMixin, which sends errors to stderr
        """
        if logging and traceback:
            logger = logging.getLogger('xmlrpc')
            if logger:
                logger.error(traceback.format_exc())

    def _dispatch(self, method, params, context=None):
        """
        Overrides SimpleXMLRPCServer to also pass server socket when calling instance's dispach.
        """

        func = None
        try:
            # check to see if a matching function has been registered
            func = self.funcs[method]
        except KeyError:
            if self.instance is not None:
                # check for a _dispatch method
                if hasattr(self.instance, '_dispatch'):
                    # also provide the socket so that socket context or peer certificate may be accessed
                    return self.instance._dispatch(method, params, context=context)
                else:
                    # call instance method directly
                    try:
                        func = resolve_dotted_attribute(
                            self.instance,
                            method,
                            self.allow_dotted_names
                        )
                    except AttributeError:
                        pass

        if func is not None:
            return func(*params)
        else:
            raise Exception('method "%s" is not supported' % method)

    def _marshaled_dispatch(self, data, dispatch_method=None, path=None, context=None):
        """
        Overrides SimpleXMLRPCDispatcher to pass connection's context to _dispatch

        Dispatches an XML-RPC method from marshalled (XML) data.

        XML-RPC methods are dispatched from the marshalled (XML) data
        using the _dispatch method and the result is returned as
        marshalled data. For backwards compatibility, a dispatch
        function can be provided as an argument (see comment in
        SimpleXMLRPCRequestHandler.do_POST) but overriding the
        existing method through subclassing is the preferred means
        of changing method dispatch behavior.
        """

        try:
            params, method = xmlrpclib.loads(data)

            # generate response
            if dispatch_method is not None:
                response = dispatch_method(method, params)
            else:
                response = self._dispatch(method, params, context=context)
            # wrap response in a singleton tuple
            response = (response,)
            response = xmlrpclib.dumps(response, methodresponse=1,
                                       allow_none=self.allow_none, encoding=self.encoding)
        except Fault, fault:
            response = xmlrpclib.dumps(fault, allow_none=self.allow_none,
                                       encoding=self.encoding)
        except:
            # report exception back to server
            exc_type, exc_value, exc_tb = sys.exc_info()
            response = xmlrpclib.dumps(
                xmlrpclib.Fault(1, "%s:%s" % (exc_type, exc_value)),
                encoding=self.encoding, allow_none=self.allow_none,
            )

        return response

class ForkingXMLRPCServer(socketserver.ForkingMixIn, SimpleXMLRPCServer):
    """
    Adds ThreadingMixin to SimpleXMLRPCServer to support multiple concurrent
    requests via forking. Also makes logging toggleable.      
    """
    def __init__(self, addr, request_handler=SilenceableXMLRPCRequestHandler, log_requests=1):
        SimpleXMLRPCServer.__init__(self, addr, request_handler, log_requests)
    

class XmlRpcHandler(object):
    """
    Base handler API for handlers used with XmlRpcNode. Public methods will be 
    exported as XML RPC methods.
    """

    def _ready(self, uri):
        """
        callback into handler to inform it of XML-RPC URI
        """
        pass
    
    def _shutdown(self, reason):
        """
        callback into handler to inform it of shutdown
        """
        pass

class XmlRpcNode(object):
    """
    Generic XML-RPC node. Handles the additional complexity of binding
    an XML-RPC server to an arbitrary port. 
    XmlRpcNode is initialized when the uri field has a value.
    """

    def __init__(self, port=0, rpc_handler=None, on_run_error=None, node_name=None, context=None):
        """
        XML RPC Node constructor
        :param port: port to use for starting XML-RPC API. Set to 0 or omit to bind to any available port, ``int``
        :param rpc_handler: XML-RPC API handler for node, `XmlRpcHandler`
        :param on_run_error: function to invoke if server.run() throws
          Exception. Server always terminates if run() throws, but this
          enables cleanup routines to be invoked if server goes down, as
          well as include additional debugging. ``fn(Exception)``
        :param node_name: needed for security policies which generate 
          certificate names based on the node name
        """
        super(XmlRpcNode, self).__init__()

        self.handler = rpc_handler
        self.uri = None # initialize the property now so it can be tested against, will be filled in later
        self.server = None
        if port and isstring(port):
            port = int(port)
        self.port = port
        self.is_shutdown = False
        self.on_run_error = on_run_error
        self.node_name = node_name 
        if node_name is None:
            raise ValueError('node_name not passed to XmlRpcNode.__init__()')
        self.context = context 
        if self.context is None:
            security.init(node_name)

    def shutdown(self, reason):
        """
        Terminate i/o connections for this server.

        :param reason: human-readable debug string, ``str``
        """
        self.is_shutdown = True
        if self.server:
            server = self.server
            handler = self.handler
            self.handler = self.server = self.port = self.uri = None
            if handler:
                handler._shutdown(reason)
            if server:
                server.socket.close()
                server.server_close()
                
    def start(self):
        """
        Initiate a thread to run the XML RPC server. Uses thread.start_new_thread.
        """
        _thread.start_new_thread(self.run, ())

    def set_uri(self, uri):
        """
        Sets the XML-RPC URI. Defined as a separate method as a hood
        for subclasses to bootstrap initialization. Should not be called externally.

        :param uri: XMLRPC URI, ``str``
        """
        self.uri = uri
        
    def run(self):
        try:
            self._run()
        except Exception as e:
            if self.is_shutdown:
                pass
            elif self.on_run_error is not None:
               self.on_run_error(e)
            else:
                raise

    def wrap_socket(self, socket):
        """
        Called whenever there is an opportunity to wrap a server socket
        """
        if self.context is None:
            return security.get().wrap_socket(socket)
        else:
            return self.context.wrap_socket(socket, server_side=True)

    def xmlrpc_protocol(self):
        """
        Called whenever there is an opportunity to get the xmlrpc_protocol
        """
        if self.context is None:
            return security.get().xmlrpc_protocol()
        else:
            return 'https'

    # separated out for easier testing
    def _run_init(self):
        logger = logging.getLogger('xmlrpc')            
        try:
            log_requests = 0
            port = self.port or 0 #0 = any

            bind_address = rosgraph.network.get_bind_address()
            logger.info("XML-RPC server binding to %s:%d" % (bind_address, port))
            
            self.server = ThreadingXMLRPCServer((bind_address, port), log_requests)
            self.port = self.server.server_address[1] #set the port to whatever server bound to
            if not self.port:
                self.port = self.server.socket.getsockname()[1] #Python 2.4

            assert self.port, "Unable to retrieve local address binding"

            # #528: semi-complicated logic for determining XML-RPC URI
            # - if ROS_IP/ROS_HOSTNAME is set, use that address
            # - if the hostname returns a non-localhost value, use that
            # - use whatever rosgraph.network.get_local_address() returns
            uri = None
            override = rosgraph.network.get_address_override()
            if override:
                uri = '%s://%s:%s/'%(self.xmlrpc_protocol(),override, self.port)
            else:
                try:
                    hostname = socket.gethostname()
                    if hostname and not hostname == 'localhost' and not hostname.startswith('127.') and hostname != '::':
                        uri = '%s://%s:%s/'%(self.xmlrpc_protocol(),hostname, self.port)
                except:
                    pass
            if not uri:
                uri = '%s://%s:%s/'%(self.xmlrpc_protocol(),rosgraph.network.get_local_address(), self.port)
            self.set_uri(uri)
            
            logger.info("Started XML-RPC server [%s]", self.uri)

            self.server.register_multicall_functions()
            self.server.register_instance(self.handler)
            self.server.socket = self.wrap_socket(self.server.socket)

        except socket.error as e:
            if e.errno == 98:
                msg = "ERROR: Unable to start XML-RPC server, port %s is already in use" % self.port
            else:
                msg = "ERROR: Unable to start XML-RPC server: %s" % e.strerror
            logger.error(msg)
            print(msg)
            raise #let higher level catch this

        if self.handler is not None:
            self.handler._ready(self.uri)
        logger.info("xml rpc node: starting XML-RPC server")
        
    def _run(self):
        """
        Main processing thread body.
        :raises: :exc:`socket.error` If server cannot bind
        
        """
        self._run_init()
        while not self.is_shutdown:
            try:
                self.server.serve_forever()
            except (IOError, select.error) as e:
                # check for interrupted call, which can occur if we're
                # embedded in a program using signals.  All other
                # exceptions break _run.
                if self.is_shutdown:
                    pass
                elif e.errno != 4:
                    self.is_shutdown = True
                    logging.getLogger('xmlrpc').error("serve forever IOError: %s, %s"%(e.errno, e.strerror))
                    
