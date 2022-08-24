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
import struct
import unittest
try:
    from cStringIO import StringIO
except ImportError:
    from io import BytesIO as StringIO
import time
import random

import rospy.rostime

try:
    import cPickle as pickle
except ImportError:
    import pickle    

# currently tests rospy.rostime, rospy.simtime, and some parts of rospy.client

class TestRospyTime(unittest.TestCase):
    
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)

    def test_import_simtime(self):
        # trip wire test, make sure the module is loading
        import rospy.impl.simtime
        # can't actually do unit tests of simtime, requires rosunit

    def test_switch_to_wallclock(self):
        rospy.rostime.switch_to_wallclock()
        self.assertAlmostEqual(time.time(), rospy.get_time(), 1)

    def test_Time_get_setstate(self):
        # use deepcopy to test getstate/setstate
        import copy, random
        a = rospy.Time(random.randint(0, 10000), random.randint(0, 10000))
        b = copy.deepcopy(a)
        self.assertEqual(a.secs, b.secs)
        self.assertEqual(a.nsecs, b.nsecs)

        buff = StringIO()
        pickle.dump(a, buff)
        buff.seek(0)
        c = pickle.load(buff)    
        self.assertEqual(a.secs, c.secs)
        self.assertEqual(a.nsecs, c.nsecs)
                                 
    def test_Duration_get_setstate(self):
        # use deepcopy to test getstate/setstate
        import copy, random
        a = rospy.Duration(random.randint(0, 10000), random.randint(0, 10000))
        b = copy.deepcopy(a)
        self.assertEqual(a.secs, b.secs)
        self.assertEqual(a.nsecs, b.nsecs)
        
        buff = StringIO()
        pickle.dump(a, buff)
        buff.seek(0)
        c = pickle.load(buff)    
        self.assertEqual(a.secs, c.secs)
        self.assertEqual(a.nsecs, c.nsecs)

    def test_Time(self):
        # This is a copy of test_roslib_rostime
        from rospy import Time, Duration
        # #1600 Duration > Time should fail
        failed = False
        try:
            v = Duration.from_sec(0.1) > Time.from_sec(0.5)
            failed = True
        except: pass
        self.assertFalse(failed, "should have failed to compare")
        try:
            v = Time.from_sec(0.4) > Duration.from_sec(0.1)
            failed = True        
        except: pass
        self.assertFalse(failed, "should have failed to compare")


        try: # neg time fails
            Time(-1)
            failed = True
        except: pass
        self.assertFalse(failed, "negative time not allowed")
        try:
            Time(1, -1000000001)
            failed = True
        except: pass
        self.assertFalse(failed, "negative time not allowed")

        # test Time.now() is within 10 seconds of actual time (really generous)
        import time
        t = time.time()
        v = Time.from_sec(t)
        self.assertEqual(v.to_sec(), t)
        # test from_sec()
        self.assertEqual(Time.from_sec(0), Time())
        self.assertEqual(Time.from_sec(1.), Time(1))
        self.assertEqual(Time.from_sec(v.to_sec()), v)
        self.assertEqual(v.from_sec(v.to_sec()), v)

        # test to_time()
        self.assertEqual(v.to_sec(), v.to_time())

        # test addition
        # - time + time fails
        try:
            v = Time(1,0) + Time(1, 0)
            failed = True
        except: pass
        self.assertFalse(failed, "Time + Time must fail")

        # - time + duration
        v = Time(1,0) + Duration(1, 0)
        self.assertEqual(Time(2, 0), v)
        v = Duration(1, 0) + Time(1,0)
        self.assertEqual(Time(2, 0), v)
        v = Time(1,1) + Duration(1, 1)
        self.assertEqual(Time(2, 2), v)
        v = Duration(1, 1) + Time(1,1)
        self.assertEqual(Time(2, 2), v)

        v = Time(1) + Duration(0, 1000000000)
        self.assertEqual(Time(2), v)
        v = Duration(1) + Time(0, 1000000000)
        self.assertEqual(Time(2), v)

        v = Time(100, 100) + Duration(300)
        self.assertEqual(Time(400, 100), v)
        v = Duration(300) + Time(100, 100)
        self.assertEqual(Time(400, 100), v)

        v = Time(100, 100) + Duration(300, 300)
        self.assertEqual(Time(400, 400), v)
        v = Duration(300, 300) + Time(100, 100)
        self.assertEqual(Time(400, 400), v)

        v = Time(100, 100) + Duration(300, -101)
        self.assertEqual(Time(399, 999999999), v)
        v =  Duration(300, -101) + Time(100, 100)
        self.assertEqual(Time(399, 999999999), v)

        # test subtraction
        try:
            v = Time(1,0) - 1
            failed = True
        except: pass
        self.assertFalse(failed, "Time - non Duration must fail")
        class Foob(object): pass      
        try:
            v = Time(1,0) - Foob()
            failed = True          
        except: pass
        self.assertFalse(failed, "Time - non TVal must fail")

        # - Time - Duration
        v = Time(1,0) - Duration(1, 0)
        self.assertEqual(Time(), v)

        v = Time(1,1) - Duration(-1, -1)
        self.assertEqual(Time(2, 2), v)
        v = Time(1) - Duration(0, 1000000000)
        self.assertEqual(Time(), v)
        v = Time(2) - Duration(0, 1000000000)
        self.assertEqual(Time(1), v)
        v = Time(400, 100) - Duration(300)
        self.assertEqual(Time(100, 100), v)
        v = Time(100, 100) - Duration(0, 101)
        self.assertEqual(Time(99, 999999999), v)

        # - Time - Time = Duration      
        v = Time(100, 100) - Time(100, 100)
        self.assertEqual(Duration(), v)
        v = Time(100, 100) - Time(100)
        self.assertEqual(Duration(0,100), v)
        v = Time(100) - Time(200)
        self.assertEqual(Duration(-100), v)

    def test_Duration(self):
        Duration = rospy.Duration

        # test from_sec
        v = Duration(1000)
        self.assertEqual(v, Duration.from_sec(v.to_sec()))
        self.assertEqual(v, v.from_sec(v.to_sec()))
        v = Duration(0,1000)
        self.assertEqual(v, Duration.from_sec(v.to_sec()))
        self.assertEqual(v, v.from_sec(v.to_sec()))
      
        # test neg
        v = -Duration(1, -1)
        self.assertEqual(-1, v.secs)
        self.assertEqual(1, v.nsecs)
        v = -Duration(-1, -1)
        self.assertEqual(1, v.secs)
        self.assertEqual(1, v.nsecs)
        v = -Duration(-1, 1)
        self.assertEqual(0, v.secs)
        self.assertEqual(999999999, v.nsecs)
      
        # test addition
        failed = False
        try:
            v = Duration(1,0) + Time(1, 0)
            failed = True
        except: pass
        self.assertFalse(failed, "Duration + Time must fail")
        try:
            v = Duration(1,0) + 1
            failed = True
        except: pass
        self.assertFalse(failed, "Duration + int must fail")
          
        v = Duration(1,0) + Duration(1, 0)
        self.assertEqual(2, v.secs)
        self.assertEqual(0, v.nsecs)
        self.assertEqual(Duration(2, 0), v)
        v = Duration(-1,-1) + Duration(1, 1)
        self.assertEqual(0, v.secs)
        self.assertEqual(0, v.nsecs)
        self.assertEqual(Duration(), v)
        v = Duration(1) + Duration(0, 1000000000)
        self.assertEqual(2, v.secs)
        self.assertEqual(0, v.nsecs)
        self.assertEqual(Duration(2), v)
        v = Duration(100, 100) + Duration(300)
        self.assertEqual(Duration(400, 100), v)
        v = Duration(100, 100) + Duration(300, 300)
        self.assertEqual(Duration(400, 400), v)
        v = Duration(100, 100) + Duration(300, -101)
        self.assertEqual(Duration(399, 999999999), v)
        
        # test subtraction
        try:
            v = Duration(1,0) - 1
            failed = True
        except: pass
        self.assertFalse(failed, "Duration - non duration must fail")
        try:
            v = Duration(1, 0) - Time(1,0)
            failed = True          
        except: pass
        self.assertFalse(failed, "Duration - Time must fail")
        
        v = Duration(1,0) - Duration(1, 0)
        self.assertEqual(Duration(), v)
        v = Duration(-1,-1) - Duration(1, 1)
        self.assertEqual(Duration(-3, 999999998), v)
        v = Duration(1) - Duration(0, 1000000000)
        self.assertEqual(Duration(), v)
        v = Duration(2) - Duration(0, 1000000000)
        self.assertEqual(Duration(1), v)
        v = Duration(100, 100) - Duration(300)
        self.assertEqual(Duration(-200, 100), v)
        v = Duration(100, 100) - Duration(300, 101)
        self.assertEqual(Duration(-201, 999999999), v)

        # test abs
        self.assertEqual(abs(Duration()), Duration())
        self.assertEqual(abs(Duration(1)), Duration(1))      
        self.assertEqual(abs(Duration(-1)), Duration(1))
        self.assertEqual(abs(Duration(0,-1)), Duration(0,1))
        self.assertEqual(abs(Duration(-1,-1)), Duration(1,1))
      
    def test_set_rostime(self):
        from rospy.rostime import _set_rostime
        from rospy import Time 

        self.assertAlmostEqual(time.time(), rospy.get_time(), 1)

        for t in [Time.from_sec(1.0), Time.from_sec(4.0)]:
            _set_rostime(t)
            self.assertEqual(t, rospy.get_rostime())
            self.assertEqual(t.to_time(), rospy.get_time())        

    def test_get_rostime(self):
        rospy.rostime.switch_to_wallclock()
        self.assertAlmostEqual(time.time(), rospy.get_time(), 1)
        self.assertAlmostEqual(time.time(), rospy.get_rostime().to_time(), 1)
        #rest of get_rostime implicitly tested by update_rostime tests

    def test_sleep(self):
        # test wallclock sleep
        rospy.rostime.switch_to_wallclock()
        rospy.sleep(0.1)
        rospy.sleep(rospy.Duration.from_sec(0.1))
        
        from rospy.rostime import _set_rostime
        from rospy import Time 

        t = Time.from_sec(1.0)
        _set_rostime(t)
        self.assertEqual(t, rospy.get_rostime())
        self.assertEqual(t.to_time(), rospy.get_time())        

        import threading

        #start sleeper
        self.assertFalse(test_sleep_done)
        sleepthread = threading.Thread(target=sleeper, args=())
        sleepthread.daemon = True
        sleepthread.start()
        time.sleep(1.0) #make sure thread is spun up
        self.assertFalse(test_sleep_done)

        t = Time.from_sec(1000000.0)
        _set_rostime(t)
        time.sleep(0.5) #give sleeper time to wakeup
        self.assertTrue(test_sleep_done, "sleeper did not wake up")

        #start duration sleeper
        self.assertFalse(test_duration_sleep_done)      
        dursleepthread = threading.Thread(target=duration_sleeper, args=())
        dursleepthread.daemon = True
        dursleepthread.start()
        time.sleep(1.0) #make sure thread is spun up
        self.assertFalse(test_duration_sleep_done)

        t = Time.from_sec(2000000.0)
        _set_rostime(t)
        time.sleep(0.5) #give sleeper time to wakeup
        self.assertTrue(test_sleep_done, "sleeper did not wake up")

        #start backwards sleeper
        self.assertFalse(test_backwards_sleep_done)
        backsleepthread = threading.Thread(target=backwards_sleeper, args=())
        backsleepthread.daemon = True
        backsleepthread.start()
        time.sleep(1.0) #make sure thread is spun up
        self.assertFalse(test_backwards_sleep_done)

        t = Time.from_sec(1.0)
        _set_rostime(t)
        time.sleep(0.5) #give sleeper time to wakeup
        self.assertTrue(test_backwards_sleep_done, "backwards sleeper was not given an exception")
    
test_duration_sleep_done = False
def duration_sleeper():
    global test_duration_sleep_done
    rospy.sleep(rospy.Duration(10000.0))
    test_duration_sleep_done = True

test_sleep_done = False
def sleeper():
    global test_sleep_done
    rospy.sleep(10000.0)
    test_sleep_done = True
test_backwards_sleep_done = False    
def backwards_sleeper():
    global test_backwards_sleep_done
    try:
        rospy.sleep(10000.0)
    except rospy.ROSException:
        test_backwards_sleep_done = True
