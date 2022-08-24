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
import unittest

import roslib
PKG='test_roslib_comm'
class MsgSpecTest(unittest.TestCase):

  def test_verbose(self):
    import roslib.msgs
    self.failIf(roslib.msgs.is_verbose())
    roslib.msgs.set_verbose(True)
    self.assertTrue(roslib.msgs.is_verbose())
    roslib.msgs.set_verbose(False)    
    self.failIf(roslib.msgs.is_verbose())
    
  def test_base_msg_type(self):
    import roslib.msgs    
    tests = [(None, None), ('String', 'String'), ('std_msgs/String', 'std_msgs/String'),
             ('String[10]', 'String'), ('string[10]', 'string'), ('std_msgs/String[10]', 'std_msgs/String'),
             ]
    for val, res in tests:
      self.assertEqual(res, roslib.msgs.base_msg_type(val))

  def test_resolve_type(self):
    from roslib.msgs import resolve_type
    for t in ['string', 'string[]', 'string[14]', 'int32', 'int32[]']:
      bt = roslib.msgs.base_msg_type(t)
      self.assertEqual(t, resolve_type(t, PKG))
      
    self.assertEqual('foo/string', resolve_type('foo/string', PKG))
    self.assertEqual('std_msgs/Header', resolve_type('Header', 'roslib'))
    self.assertEqual('std_msgs/Header', resolve_type('std_msgs/Header', 'roslib'))
    self.assertEqual('std_msgs/Header', resolve_type('Header', 'stereo_msgs'))
    self.assertEqual('std_msgs/String', resolve_type('String', 'std_msgs'))    
    self.assertEqual('std_msgs/String', resolve_type('std_msgs/String', 'std_msgs'))
    self.assertEqual('std_msgs/String', resolve_type('std_msgs/String', PKG))    
    self.assertEqual('std_msgs/String[]', resolve_type('std_msgs/String[]', PKG))
    
  def test_parse_type(self):
    tests = [
      ('a', ('a', False, None)),
      ('int8', ('int8', False, None)),      
      ('std_msgs/String', ('std_msgs/String', False, None)),
      ('a[]', ('a', True, None)),
      ('int8[]', ('int8', True, None)),      
      ('std_msgs/String[]', ('std_msgs/String', True, None)),
      ('a[1]', ('a', True, 1)),
      ('int8[1]', ('int8', True, 1)),      
      ('std_msgs/String[1]', ('std_msgs/String', True, 1)),
      ('a[11]', ('a', True, 11)),
      ('int8[11]', ('int8', True, 11)),      
      ('std_msgs/String[11]', ('std_msgs/String', True, 11)),
      ]
    for val, res in tests:
      self.assertEqual(res, roslib.msgs.parse_type(val))
    
    fail = ['a[1][2]', 'a[][]', '', None, 'a[', 'a[[1]', 'a[1]]']
    for f in fail:
      try:
        roslib.msgs.parse_type(f)
        self.fail("should have failed on " +f)
      except roslib.msgs.MsgSpecException: pass

  def test_Constant(self):
    import random
    import roslib.msgs    
    vals = [random.randint(0, 1000) for i in range(0, 3)]
    type_, name, val = [str(x) for x in vals]
    x = roslib.msgs.Constant(type_, name, val, str(val))
    self.assertEqual(type_, x.type)
    self.assertEqual(name, x.name)
    self.assertEqual(val, x.val)
    self.assertEqual(roslib.msgs.Constant(type_, name, val, str(val)), x)

    self.assertNotEquals(1, x)
    self.assertNotEquals(roslib.msgs.Constant('baz', name, val, str(val)), x)
    self.assertNotEquals(roslib.msgs.Constant(type_, 'foo', val, str(val)), x)
    self.assertNotEquals(roslib.msgs.Constant(type_, name, 'foo', 'foo'), x)

    # tripwire
    self.assertTrue(repr(x))
    self.assertTrue(str(x))    
    
    try:
      roslib.msgs.Constant(None, name, val, str(val))
    except: pass
    try:
      roslib.msgs.Constant(type_, None, val, str(val))
    except: pass
    try:
      roslib.msgs.Constant(type_, name, None, 'None')      
    except: pass
    try:
      roslib.msgs.Constant(type_, name, val, None)      
    except: pass
    
    try:
      x.foo = 'bar'
      self.fail('Constant should not allow arbitrary attr assignment')
    except: pass
    
  def test_MsgSpec(self):
    def sub_test_MsgSpec(types, names, constants, text, has_header):
      m = MsgSpec(types, names, constants, text)
      self.assertEqual(m.types, types)
      self.assertEqual(m.names, names)
      self.assertEqual(m.text, text)
      self.assertEqual(has_header, m.has_header())
      self.assertEqual(m.constants, constants)
      self.assertEqual(list(zip(types, names)), m.fields())
      self.assertEqual(m, MsgSpec(types, names, constants, text))
      return m
    
    from roslib.msgs import MsgSpec
    # allow empty msg
    empty = sub_test_MsgSpec([], [], [], '', False)
    self.assertEqual([], empty.fields())    

    # one-field
    one_field = sub_test_MsgSpec(['int32'], ['x'], [], 'int32 x', False)
    # make sure that equals tests every declared field
    self.assertEqual(one_field, MsgSpec(['int32'], ['x'], [], 'int32 x'))
    self.assertNotEquals(one_field, MsgSpec(['uint32'], ['x'], [], 'int32 x'))
    self.assertNotEquals(one_field, MsgSpec(['int32'], ['y'], [], 'int32 x'))
    self.assertNotEquals(one_field, MsgSpec(['int32'], ['x'], [], 'uint32 x'))
    # test against __ne__ as well
    self.assertTrue(one_field != MsgSpec(['int32'], ['x'], [], 'uint32 x'))
    #test strify
    self.assertEqual("int32 x", str(one_field).strip())
    
    # test variations of multiple fields and headers
    two_fields = sub_test_MsgSpec(['int32', 'string'], ['x', 'str'], [], 'int32 x\nstring str', False)
    one_header = sub_test_MsgSpec(['Header'], ['header'], [], 'Header header', True)
    header_and_fields = sub_test_MsgSpec(['Header', 'int32', 'string'], ['header', 'x', 'str'], [], 'Header header\nint32 x\nstring str', True)
    embed_types = sub_test_MsgSpec(['Header', 'std_msgs/Int32', 'string'], ['header', 'x', 'str'], [], 'Header header\nstd_msgs/Int32 x\nstring str', True)
    #test strify
    self.assertEqual("int32 x\nstring str", str(two_fields).strip())

    # types and names mismatch
    try:
      MsgSpec(['int32', 'int32'], ['intval'], [], 'int32 intval\int32 y')
      self.fail("types and names must align")
    except: pass

    # test (not) equals against non msgspec
    self.failIf(one_field == 1)
    self.assertTrue(one_field != 1)    
    #TODO: test flatten

    # test that repr doesn't throw an error
    [repr(x) for x in [empty, one_field, one_header, two_fields, embed_types]]

  def test_init(self):
    import roslib.msgs    
    roslib.msgs._initialized = False
    roslib.msgs._init()
    self.assertTrue(roslib.msgs._initialized)
    # test repeated initialization
    roslib.msgs._init()    

  def test___convert_val(self):
    from roslib.msgs import _convert_val, MsgSpecException
    self.assertEqual(0., _convert_val('float32', '0.0'))
    self.assertEqual(0., _convert_val('float64', '0.0'))
    
    self.assertEqual('fo o', _convert_val('string', '   fo o '))

    self.assertEqual(1, _convert_val('byte', '1'))
    self.assertEqual(1, _convert_val('char', '1'))
    self.assertEqual(1, _convert_val('int8', '1'))
    self.assertEqual(12, _convert_val('int16', '12'))
    self.assertEqual(-13, _convert_val('int32', '-13'))
    self.assertEqual(14, _convert_val('int64', '14'))
    self.assertEqual(0, _convert_val('uint8', '0'))
    self.assertEqual(18, _convert_val('uint16', '18'))
    self.assertEqual(19, _convert_val('uint32', '19'))    
    self.assertEqual(20, _convert_val('uint64', '20'))

    width_fail = [('int8', '129'), ('uint8', '256'),
                  ('int16', '35536'), ('uint16', '-1'),('uint16', '65536'),
                  ('int32', '3000000000'),('int32', '-2700000000'),
                  ('uint32', '-1'),('uint32', '41000000000'),
                  ('uint64', '-1')]
    for t, v in width_fail:
      try:
        _convert_val(t, v)
        self.fail("should have failed width check: %s, %s"%(t, v))
      except MsgSpecException: pass
    type_fail = [('int32', 'f'), ('float32', 'baz')]
    for t, v in type_fail:
      try:
        _convert_val(t, v)
        self.fail("should have failed type check: %s, %s"%(t, v))
      except ValueError: pass
    try:
      _convert_val('foo', '1')
      self.fail("should have failed invalid type")
    except MsgSpecException: pass
    
  def test_msg_file(self):
    import roslib.msgs    
    f = os.path.normcase(roslib.msgs.msg_file('rosgraph_msgs', 'Log'))
    self.assertTrue(os.path.isfile(f))
    self.assertTrue(f.endswith(os.path.normcase('rosgraph_msgs/msg/Log.msg')))

    # msg_file should return paths even for non-existent resources
    f = os.path.normcase(roslib.msgs.msg_file('roslib', 'Fake'))
    self.failIf(os.path.isfile(f))
    self.assertTrue(f.endswith(os.path.normcase('roslib/msg/Fake.msg')))

  def test_is_valid_msg_type(self):
    import roslib.msgs
    vals = [
      #basic
      'F', 'f', 'Foo', 'Foo1',
      'std_msgs/String',
      # arrays
      'Foo[]', 'Foo[1]', 'Foo[10]',
      ]
    for v in vals:
      self.assertTrue(roslib.msgs.is_valid_msg_type(v), "roslib.msgs.is_valid_msg_type should have returned True for '%s'"%v)
 
    # bad cases
    vals = [None, '', '#', '%', 'Foo%', 'Woo Woo',
            '/', '/String', 
            'Foo[f]', 'Foo[1d]', 'Foo[-1]', 'Foo[1:10]', 'Foo[', 'Foo]', 'Foo[]Bar']
    for v in vals:
      self.failIf(roslib.msgs.is_valid_msg_type(v), "roslib.msgs.is_valid_msg_type should have returned False for '%s'"%v)
      
  def test_is_valid_constant_type(self):
    import roslib.msgs
    valid = ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', \
             'uint64', 'float32', 'float64', 'char', 'byte', 'string']
    invalid = [
      'std_msgs/String', '/', 'String',
      'time', 'duration','header',
    ]
    for v in valid:
      self.assertTrue(roslib.msgs.is_valid_constant_type(v), "roslib.msgs.is_valid_constant_type should have returned True for '%s'"%v)
    for v in invalid:
      self.failIf(roslib.msgs.is_valid_constant_type(v), "roslib.msgs.is_valid_constant_type should have returned False for '%s'"%v)
    
