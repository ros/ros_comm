#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


## Integration test for empty services to test serializers
## and transport

PKG = 'test_rospy'
NAME = 'basic_services'

import sys 
import time
import math
import unittest

import rospy
import rostest
from test_rospy.srv import *

CONSTANTS_SERVICE_NAKED = 'constants_service_naked'
CONSTANTS_SERVICE_WRAPPED = 'constants_service_wrapped'

ADD_TWO_INTS_SERVICE_NAKED = 'a2i_naked'
ADD_TWO_INTS_SERVICE_WRAPPED = 'a2i_wrapped'

STRING_CAT_SERVICE_NAKED = 'string_lower_naked'
STRING_CAT_SERVICE_WRAPPED = 'string_lower_wrapped'

FAULTY_SERVICE = 'faulty_service'
FAULTY_SERVICE_RESULT = 'faulty_service_result'

#TODO:

STRING_SERVICE       = 'string_service'
EMBEDDED_MSG_SERVICE = 'embedded_msg_service'

WAIT_TIMEOUT = 10.0 #s

def add_two_ints_wrapped(req):
    from test_rosmaster.srv import AddTwoIntsResponse
    return AddTwoIntsResponse(req.a + req.b)
def add_two_ints_naked(req):
    return req.a + req.b

def string_cat_naked(req):
    from std_msgs.msg import String
    return String(req.str.data + req.str2.val)
def string_cat_wrapped(req):
    from test_rospy.srv import StringStringResponse
    from std_msgs.msg import String
    return StringStringResponse(String(req.str.data + req.str2.val))

def handle_constants_wrapped(req):
    cmr = ConstantsMultiplexRequest
    Resp = ConstantsMultiplexResponse
    if req.selection == cmr.SELECT_X:
        # test w/o wrapper
        return Resp(req.selection,
                    cmr.BYTE_X, cmr.INT32_X, cmr.UINT32_X, cmr.FLOAT32_X)
    elif req.selection == cmr.SELECT_Y:
        return Resp(req.selection,
            cmr.BYTE_Y, cmr.INT32_Y, cmr.UINT32_Y, cmr.FLOAT32_Y)
    elif req.selection == cmr.SELECT_Z:
        return Resp(req.selection,
            cmr.BYTE_Z, cmr.INT32_Z, cmr.UINT32_Z, cmr.FLOAT32_Z)
    else:
        print("test failed, req.selection not in (X,Y,Z)", req.selection)

def handle_constants_naked(req):
    cmr = ConstantsMultiplexRequest
    if req.selection == cmr.SELECT_X:
        return req.selection,cmr.BYTE_X, cmr.INT32_X, cmr.UINT32_X, cmr.FLOAT32_X
    elif req.selection == cmr.SELECT_Y:
        return req.selection, cmr.BYTE_Y, cmr.INT32_Y, cmr.UINT32_Y, cmr.FLOAT32_Y
    elif req.selection == cmr.SELECT_Z:
        return req.selection, cmr.BYTE_Z, cmr.INT32_Z, cmr.UINT32_Z, cmr.FLOAT32_Z
    else:
        print("test failed, req.selection not in (X,Y,Z)", req.selection)

class UnexpectedException(Exception):
    pass


class FaultyHandler(object):
    def __init__(self):
        self.test_call = False

    def custom_error_handler(self, e, exc_type, exc_value, tb):
        self.test_call = True

    def call_handler(self, req):
        raise UnexpectedException('Call raised an exception')

    def result_handler(self, req):
        resp = EmptyReqSrvResponse()
        resp.fake_secret = 1 if self.test_call else 0
        return resp


def services():
    from test_rosmaster.srv import AddTwoInts
    rospy.init_node(NAME)
    s1 = rospy.Service(CONSTANTS_SERVICE_NAKED, ConstantsMultiplex, handle_constants_naked)
    s2 = rospy.Service(CONSTANTS_SERVICE_WRAPPED, ConstantsMultiplex, handle_constants_wrapped)
    
    s3 = rospy.Service(ADD_TWO_INTS_SERVICE_NAKED, AddTwoInts, add_two_ints_naked)
    s4 = rospy.Service(ADD_TWO_INTS_SERVICE_WRAPPED, AddTwoInts, add_two_ints_wrapped)

    s5 = rospy.Service(STRING_CAT_SERVICE_NAKED, StringString, string_cat_naked)
    s6 = rospy.Service(STRING_CAT_SERVICE_WRAPPED, StringString, string_cat_wrapped)

    faulty_handler = FaultyHandler()
    s7 = rospy.Service(FAULTY_SERVICE, EmptySrv, faulty_handler.call_handler, error_handler=faulty_handler.custom_error_handler)
    s8 = rospy.Service(FAULTY_SERVICE_RESULT, EmptyReqSrv, faulty_handler.result_handler)

    rospy.spin()

class TestBasicServicesClient(unittest.TestCase):

    # call with request instance style
    def _test(self, name, srv, req):
        rospy.wait_for_service(name, WAIT_TIMEOUT)        
        s = rospy.ServiceProxy(name, srv)
        resp = s.call(req)
        self.assert_(resp is not None)
        return resp

    # call with args style    
    def _test_req_naked(self, name, srv, args):
        rospy.wait_for_service(name, WAIT_TIMEOUT)        
        s = rospy.ServiceProxy(name, srv)
        resp = s.call(*args)
        self.assert_(resp is not None)
        return resp

    # call with keyword style
    def _test_req_kwds(self, name, srv, kwds):
        rospy.wait_for_service(name, WAIT_TIMEOUT)        
        s = rospy.ServiceProxy(name, srv)
        resp = s.call(**kwds)
        self.assert_(resp is not None)
        return resp
    
    def test_calltype_mismatch(self):
        try:
            s = rospy.ServiceProxy(CONSTANTS_SERVICE_WRAPPED, ConstantsMultiplex)
            s.call(EmptySrvRequest())
            self.fail("rospy failed to raise TypeError when request type does not match service type")
        except TypeError:
            pass
        
    def test_type_mismatch(self):
        try:
            self._test(CONSTANTS_SERVICE_WRAPPED, EmptySrv, EmptySrvRequest())
            self.fail("Service failed to throw exception on type mismatch [sent EmptySrvRequest to ConstantsMultiplex service")
        except rospy.ServiceException:
            pass 

    def test_add_two_ints(self):
        # add two ints checks single, integer return value, which is
        # an interesting in the naked case
        from test_rosmaster.srv import AddTwoInts, AddTwoIntsRequest
        Cls = AddTwoInts
        Req = AddTwoIntsRequest

        for name in [ADD_TWO_INTS_SERVICE_NAKED, ADD_TWO_INTS_SERVICE_WRAPPED]:
            resp_req = self._test(name, Cls, Req(1, 2))
            resp_req_naked = self._test_req_naked(name, Cls, (1, 2))
            resp_req_kwds = self._test_req_kwds(name, Cls, {'a': 3})
            for resp in [resp_req, resp_req_naked, resp_req_kwds]:
                self.assertEquals(3, resp.sum)

    def test_String_String(self):
        from std_msgs.msg import String
        from test_rospy.srv import StringString, StringStringRequest
        from test_rospy.msg import Val
        Cls = StringString
        Req = StringStringRequest

        for name in [STRING_CAT_SERVICE_NAKED, STRING_CAT_SERVICE_WRAPPED]:
            resp_req = self._test(name, Cls, Req(String('FOO'), Val('bar')))
            resp_req_naked = self._test_req_naked(name, Cls, (String('FOO'), Val('bar'),))
            resp_req_kwds = self._test_req_kwds(name, Cls, {'str': String('FOO'), 'str2': Val('bar')})
            for resp in [resp_req, resp_req_naked, resp_req_kwds]:
                self.assertEquals('FOObar', resp.str.data)

    def test_String_String_unicode(self):
        from std_msgs.msg import String
        from test_rospy.srv import StringString, StringStringRequest
        from test_rospy.msg import Val
        Cls = StringString
        Req = StringStringRequest

        for name in [STRING_CAT_SERVICE_NAKED, STRING_CAT_SERVICE_WRAPPED]:
            resp_req = self._test(name, Cls, Req(String(u'ロボット'), Val(u'机器人')))
            resp_req_naked = self._test_req_naked(name, Cls, (String(u'ロボット'), Val(u'机器人'),))
            resp_req_kwds = self._test_req_kwds(name, Cls, {'str': String(u'ロボット'), 'str2': Val(u'机器人')})
            for resp in [resp_req, resp_req_naked, resp_req_kwds]:
                self.assertEquals('ロボット机器人', resp.str.data)  # if you send in unicode, you'll receive in str

    def test_constants(self):
        Cls = ConstantsMultiplex
        Req = ConstantsMultiplexRequest

        for name in [CONSTANTS_SERVICE_NAKED, CONSTANTS_SERVICE_WRAPPED]:
            # test all three call forms
            resp_req = self._test(name, Cls, Req(Req.SELECT_X))
            resp_req_naked = self._test_req_naked(name, Cls, (Req.SELECT_X,))
            resp_req_kwds = self._test_req_kwds(name, Cls, {'selection': Req.SELECT_X})

            for resp in [resp_req, resp_req_naked, resp_req_kwds]:
                self.assertEquals(ConstantsMultiplexResponse.CONFIRM_X,
                                  resp.select_confirm)
                self.assertEquals(Req.BYTE_X, resp.ret_byte)
                self.assertEquals(Req.INT32_X, resp.ret_int32)
                self.assertEquals(Req.UINT32_X, resp.ret_uint32)
                self.assert_(math.fabs(Req.FLOAT32_X - resp.ret_float32) < 0.001)

            resp = self._test(name, Cls,
                              ConstantsMultiplexRequest(Req.SELECT_Y))
            self.assertEquals(ConstantsMultiplexResponse.CONFIRM_Y,
                              resp.select_confirm)
            self.assertEquals(Req.BYTE_Y, resp.ret_byte)
            self.assertEquals(Req.INT32_Y, resp.ret_int32)
            self.assertEquals(Req.UINT32_Y, resp.ret_uint32)
            self.assert_(math.fabs(Req.FLOAT32_Y - resp.ret_float32) < 0.001)

            resp = self._test(name, Cls,
                              ConstantsMultiplexRequest(Req.SELECT_Z))
            self.assertEquals(ConstantsMultiplexResponse.CONFIRM_Z,
                              resp.select_confirm)
            self.assertEquals(Req.BYTE_Z, resp.ret_byte)
            self.assertEquals(Req.INT32_Z, resp.ret_int32)
            self.assertEquals(Req.UINT32_Z, resp.ret_uint32)
            self.assert_(math.fabs(Req.FLOAT32_Z - resp.ret_float32) < 0.001)

    def test_faulty_service(self):
        rospy.wait_for_service(FAULTY_SERVICE, WAIT_TIMEOUT)
        sproxy = rospy.ServiceProxy(FAULTY_SERVICE, EmptySrv)

        rospy.wait_for_service(FAULTY_SERVICE_RESULT, WAIT_TIMEOUT)
        sproxy_result = rospy.ServiceProxy(FAULTY_SERVICE_RESULT, EmptyReqSrv)

        resp = sproxy_result.call(EmptyReqSrvRequest())
        self.assertEquals(resp.fake_secret, 0)
        try:
            resp = sproxy.call(EmptySrvRequest())
            self.assert_(False)
        except rospy.ServiceException:
            pass
        resp = sproxy_result.call(EmptyReqSrvRequest())
        self.assertEquals(resp.fake_secret, 1)

if __name__ == '__main__':
    if '--service' in sys.argv:
        services()
    else:
        rostest.run(PKG, 'rospy_basic_services', TestBasicServicesClient, sys.argv)
