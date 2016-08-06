from __future__ import print_function

import os
import sys
import logging
import threading
import time
import traceback

# from rosgraph.xmlrpc import XmlRpcHandler
# 
# import rosgraph.names
# from rosgraph.names import resolve_name
# import rosgraph.security as security
# import rosmaster.paramserver
# import rosmaster.threadpool
# 
# from rosmaster.util import xmlrpcapi
# from rosmaster.registrations import RegistrationManager
from rosmaster.validators import non_empty, non_empty_str, not_none, is_api, is_topic, is_service, valid_type_name, \
    valid_name, empty_or_valid_name, ParameterInvalid

_logger = logging.getLogger("rosmaster.keyserver")

LOG_API = False


def mloginfo(msg, *args):
    """
    Info-level master log statements. These statements may be printed
    to screen so they should be user-readable.
    @param msg: Message string
    @type  msg: str
    @param args: arguments for msg if msg is a format string
    """
    # mloginfo is in core so that it is accessible to master and masterdata
    _logger.info(msg, *args)


def mlogwarn(msg, *args):
    """
    Warn-level master log statements. These statements may be printed
    to screen so they should be user-readable.
    @param msg: Message string
    @type  msg: str    
    @param args: arguments for msg if msg is a format string
    """
    # mloginfo is in core so that it is accessible to master and masterdata
    _logger.warn(msg, *args)
    if args:
        print("WARN: " + msg % args)
    else:
        print("WARN: " + str(msg))


def apivalidate(error_return_value, validators=()):
    """
    ROS master/slave arg-checking decorator. Applies the specified
    validator to the corresponding argument and also remaps each
    argument to be the value returned by the validator.  Thus,
    arguments can be simultaneously validated and canonicalized prior
    to actual function call.
    @param error_return_value: API value to return if call unexpectedly fails
    @param validators: sequence of validators to apply to each
      arg. None means no validation for the parameter is required. As all
      api methods take caller_id as the first parameter, the validators
      start with the second param.
    @type  validators: sequence
    """

    def check_validates(f):
        try:
            func_code = f.__code__
            func_name = f.__name__
        except AttributeError:
            func_code = f.func_code
            func_name = f.func_name
        assert len(validators) == func_code.co_argcount - 2, "%s failed arg check" % f  # ignore self and caller_id

        def validated_f(*args, **kwds):
            if LOG_API:
                _logger.debug("%s%s", func_name, str(args[1:]))
                # print "%s%s"%(func_name, str(args[1:]))
            if len(args) == 1:
                _logger.error("%s invoked without caller_id paramter" % func_name)
                return -1, "missing required caller_id parameter", error_return_value
            elif len(args) != func_code.co_argcount:
                return -1, "Error: bad call arity", error_return_value

            instance = args[0]
            caller_id = args[1]

            def isstring(s):
                """Small helper version to check an object is a string in
                a way that works for both Python 2 and 3
                """
                try:
                    return isinstance(s, basestring)
                except NameError:
                    return isinstance(s, str)

            if not isstring(caller_id):
                _logger.error("%s: invalid caller_id param type", func_name)
                return -1, "caller_id must be a string", error_return_value

            newArgs = [instance, caller_id]  # canonicalized args
            try:
                for (v, a) in zip(validators, args[2:]):
                    if v:
                        try:
                            newArgs.append(v(a, caller_id))
                        except ParameterInvalid as e:
                            _logger.error("%s: invalid parameter: %s", func_name, str(e) or 'error')
                            return -1, str(e) or 'error', error_return_value
                    else:
                        newArgs.append(a)

                if LOG_API:
                    retval = f(*newArgs, **kwds)
                    _logger.debug("%s%s returns %s", func_name, args[1:], retval)
                    return retval
                else:
                    code, msg, val = f(*newArgs, **kwds)
                    if val is None:
                        return -1, "Internal error (None value returned)", error_return_value
                    return code, msg, val
            except TypeError as te:  # most likely wrong arg number
                _logger.error(traceback.format_exc())
                return -1, "Error: invalid arguments: %s" % te, error_return_value
            except Exception as e:  # internal failure
                _logger.error(traceback.format_exc())
                return 0, "Internal failure: %s" % e, error_return_value

        try:
            validated_f.__name__ = func_name
        except AttributeError:
            validated_f.func_name = func_name
        validated_f.__doc__ = f.__doc__  # preserve doc
        return validated_f

    return check_validates


###################################################
# Master Implementation

class KeyserverHandler(object):
    """
    XML-RPC handler for ROS Keyserver APIs.
    API routines for the ROS Keyserver Node.

    By convention, ROS nodes take in caller_id as the first parameter
    of any API call.  The setting of this parameter is rarely done by
    client code as ros::msproxy::MasterProxy automatically inserts
    this parameter (see ros::client::getMaster()).
    """

    def __init__(self, key_helper):
        """ctor."""

        self.key_helper = key_helper
        
        self.uri = None
        self.done = False


    def _shutdown(self, reason=''):
        self.done = True

    def _ready(self, uri):
        """
        Initialize the handler with the XMLRPC URI. This is a standard callback from the XmlRpcNode API.

        @param uri: XML-RPC URI
        @type  uri: str
        """
        self.uri = uri

    def _ok(self):
        return not self.done

    ###############################################################################
    # EXTERNAL API

    @apivalidate(0, (None,))
    def shutdown(self, caller_id, msg=''):
        """
        Stop this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param msg: a message describing why the node is being shutdown.
        @type  msg: str
        @return: [code, msg, 0]
        @rtype: [int, str, int]
        """
        if msg:
            print("shutdown request: %s" % msg, file=sys.stdout)
        else:
            print("shutdown requst", file=sys.stdout)
        self._shutdown('external shutdown request from [%s]: %s' % (caller_id, msg))
        return 1, "shutdown", 0

    @apivalidate('')
    def getUri(self, caller_id):
        """
        Get the XML-RPC URI of this server.
        @param caller_id str: ROS caller id    
        @return [int, str, str]: [1, "", xmlRpcUri]
        """
        return 1, "", self.uri

    @apivalidate(-1)
    def getPid(self, caller_id):
        """
        Get the PID of this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: [1, "", serverProcessPID]
        @rtype: [int, str, int]
        """
        return 1, "", os.getpid()

    ##################################################################################
    # KEYSERVER APIS


    @apivalidate('')
    def getCA(self, caller_id):
        """
        Get the Certificate Authorities of the master keystore.
        This API is for getting public certificates of trusted CAs 
        within the master keystore so that a chain of trust may be evaluated.
        Returns a dict structure where keys as certificate's filename as a string,
        and values are the PEM encoding of the certificate file as a string.
        Use this API to bootstrap a node's CA path within a new keystore, 
        one that may be separate to that of the master's, 
        to establish a chain of trust to validate SROS certificates.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: (code, msg, CAs)
        @rtype: (int, str, {str:str, str:str,}
        """
        if self._ok():
            return 1, "certificate authority certs", self.key_helper.get_ca()
        else:
            return -1, "keyserver has been shutdown", 0

    @apivalidate(0, (valid_name('node_stem'),))
    def requestNodeStore(self, caller_id, node_stem):
        """
        Request new Node Store keyserver given a node stem.
        This API is for getting keypairs for a node store singed by trusted CAs 
        within the master keystore so that a chain of trust may be evaluated.
        Returns a dict structure where keys as key/cert filename as a string,
        and values are the PEM encoding of the key/cert file as a string.
        Use this API to bootstrap a node's node store within a new keystore
        to establish a chain of trust to validate SROS certificates.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: (code, msg, CAs)
        @rtype: (int, str, {str:str, str:str,}
        """
        if self._ok():
            return 1, "Nodestore for [%s]" % node_stem, self.key_helper.get_nodestore(node_stem)
        else:
            return -1, "keyserver has been shutdown", 0
