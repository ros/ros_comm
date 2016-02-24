#!/usr/bin/env python
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

import os
import sys
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import struct
import unittest
import time

try:
    import numpy
    disable = False
except ImportError:
    print("cannot import numpy, test is disabled")
    disable = True

# this is partially a teste of the rospy/Tutorials/numpy
from test_rospy.msg import Floats

# test rospy.names package
class TestRospyNumpy(unittest.TestCase):

    def test_floats(self):
        if disable:
            return
        vals = [1.0, 2.1, 3.2, 4.3, 5.4, 6.5]
        b = StringIO()
        f = Floats(numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32))
        f.serialize(b)

        # deserialize twice, once with numpy wrappers, once without
        f2 = Floats()
        self.assert_(type(f2.data) == list)
        f2.deserialize(b.getvalue())
        for x, y in zip(f2.data, vals):
            self.assertAlmostEquals(x, y, 2)

        from rospy.numpy_msg import numpy_msg
        f3 = numpy_msg(Floats)()
        if 0:
            # future
            self.assert_(isinstance(f3.data, numpy.ndarray), type(f3.data))
        f3.deserialize(b.getvalue())
        self.assert_(isinstance(f3.data, numpy.ndarray), type(f3.data))
        v = numpy.equal(f3.data, numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32))
        self.assert_(v.all())

    def test_class_identity(self):
        from rospy.numpy_msg import numpy_msg
        self.assert_(isinstance(numpy_msg(Floats)(), numpy_msg(Floats)))

        FloatsNP = numpy_msg(Floats)
        FloatsNP2 = numpy_msg(Floats)

        self.assert_(FloatsNP is FloatsNP2)
