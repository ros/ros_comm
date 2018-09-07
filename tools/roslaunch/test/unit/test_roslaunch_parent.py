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
import unittest
import time

import rospkg
import rosgraph.network
import roslaunch.parent

import rosgraph
master = rosgraph.Master('test_roslaunch_parent')
def get_param(*args):
    return master.getParam(*args)

## Fake Process object
class ProcessMock(roslaunch.pmon.Process):
    def __init__(self, package, name, args, env, respawn=False):
        super(ProcessMock, self).__init__(package, name, args, env, respawn)
        self.stopped = False
    def stop(self):
        self.stopped = True

## Fake ProcessMonitor object
class ProcessMonitorMock(object):
    def __init__(self):
        self.core_procs = []
        self.procs = []
        self.listeners = []
        self.is_shutdown = False
        
    def join(self, timeout=0):
        pass

    def add_process_listener(self, l):
        self.listeners.append(l)

    def register(self, p):
        self.procs.append(p)

    def register_core_proc(self, p):
        self.core_procs.append(p)
        
    def registrations_complete(self):
        pass
        
    def unregister(self, p):
        self.procs.remove(p)

    def has_process(self, name):
        return len([p for p in self.procs if p.name == name]) > 0

    def get_process(self, name):
        val = [p for p in self.procs if p.name == name]
        if val:
            return val[0]
        return None

    def has_main_thread_jobs(self):
        return False
    
    def do_main_thread_jobs(self):
        pass
    
    def kill_process(self, name):
        pass
        
    def shutdown(self):
        self.is_shutdown = True
        
    def get_active_names(self):
        return [p.name for p in self.procs]

    def get_process_names_with_spawn_count(self):
        actives = [(p.name, p.spawn_count) for p in self.procs]
        deads = []
        return [actives, deads]

    def mainthread_spin_once(self):
        pass
        
    def mainthread_spin(self):
        pass

    def run(self):
        pass
            
## Test roslaunch.server
class TestRoslaunchParent(unittest.TestCase):

    def setUp(self):
        self.pmon = ProcessMonitorMock()

    def test_roslaunchParent(self):
        try:
            self._subroslaunchParent()
        finally:
            self.pmon.shutdown()

    def _subroslaunchParent(self):
        from roslaunch.parent import ROSLaunchParent
        pmon = self.pmon
        try:
            # if there is a core up, we have to use its run id
            run_id = get_param('/run_id')
        except:
            run_id = 'test-rl-parent-%s'%time.time()
        name = 'foo-bob'
        server_uri = 'http://localhost:12345'
        
        p = ROSLaunchParent(run_id, [], is_core = True, port=None, local_only=False)
        self.assertEquals(run_id, p.run_id)
        self.assertEquals(True, p.is_core)
        self.assertEquals(False, p.local_only)

        rl_dir = rospkg.RosPack().get_path('roslaunch')
        rl_file = os.path.join(rl_dir, 'resources', 'example.launch')
        self.assert_(os.path.isfile(rl_file))
        
        # validate load_config logic
        p = ROSLaunchParent(run_id, [rl_file], is_core = False, port=None, local_only=True)
        self.assertEquals(run_id, p.run_id)
        self.assertEquals(False, p.is_core)
        self.assertEquals(True, p.local_only)

        self.assert_(p.config is None)
        p._load_config()
        self.assert_(p.config is not None)
        self.assert_(p.config.nodes)

        # try again with port override
        p = ROSLaunchParent(run_id, [rl_file], is_core = False, port=11312, local_only=True)
        self.assertEquals(11312, p.port)
        self.assert_(p.config is None)
        p._load_config()
        # - make sure port got passed into master
        _, port = rosgraph.network.parse_http_host_and_port(p.config.master.uri)
        self.assertEquals(11312, port)

        # try again with bad file
        p = ROSLaunchParent(run_id, ['non-existent-fake.launch'])
        self.assert_(p.config is None)
        try:
            p._load_config()
            self.fail("load config should have failed due to bad rl file")
        except roslaunch.core.RLException: pass

        # try again with bad xml
        rl_dir = rospkg.RosPack().get_path('roslaunch')
        rl_file = os.path.join(rl_dir, 'test', 'xml', 'test-params-invalid-1.xml')
        self.assert_(os.path.isfile(rl_file))
        p = ROSLaunchParent(run_id, [rl_file])
        self.assert_(p.config is None)
        try:
            p._load_config()
            self.fail("load config should have failed due to bad rl file")
        except roslaunch.core.RLException: pass
        
        # Mess around with internal repr to get code coverage on _init_runner/_init_remote
        p = ROSLaunchParent(run_id, [], is_core = False, port=None, local_only=True)
        # no config, _init_runner/_init_remote/_start_server should fail
        for m in ['_init_runner', '_init_remote', '_start_server']:
            try:
                getattr(p, m)()
                self.fail('should have raised')
            except roslaunch.core.RLException: pass

        # - initialize p.config
        p.config = roslaunch.config.ROSLaunchConfig()
        
        # no pm, _init_runner/_init_remote/_start_server should fail
        for m in ['_init_runner', '_init_remote', '_start_server']:
            try:
                getattr(p, m)()
                self.fail('should have raised')
            except roslaunch.core.RLException: pass

        # - initialize p.pm
        p.pm = pmon
        
        for m in ['_init_runner', '_init_remote']:
            try:
                getattr(p, m)()
                self.fail('should have raised')
            except roslaunch.core.RLException: pass
            
        from roslaunch.server import ROSLaunchParentNode
        p.server = ROSLaunchParentNode(p.config, pmon)
        p._init_runner()
        # roslaunch runner should be initialized
        self.assert_(p.runner is not None)

        # test _init_remote
        p.local_only = True
        p._init_remote()
        p.local_only = False
        # - this violates many abstractions to do this
        def ftrue():
            return True
        p.config.has_remote_nodes = ftrue
        p._init_remote()
        self.assert_(p.remote_runner is not None)

        self.failIf(pmon.is_shutdown)
        p.shutdown()
        self.assert_(pmon.is_shutdown)        


## Test sigint_timeout and sigterm_timeout
# We need real ProcessMonitor here, and we need to spawn real threads
class TestRoslaunchTimeouts(unittest.TestCase):
    def setUp(self):
        from roslaunch.pmon import ProcessMonitor
        self.pmon = ProcessMonitor()

    def test_roslaunchTimeouts(self):
        try:
            self._subtestTimeouts()
        finally:
            self.pmon.shutdown()

    def _subtestTimeouts(self):
        from roslaunch.parent import ROSLaunchParent
        from roslaunch.server import ROSLaunchParentNode
        import signal
        import tempfile
        from threading import Thread

        pmon = self.pmon
        pmon.start()
        try:
            # if there is a core up, we have to use its run id
            run_id = get_param('/run_id')
        except:
            run_id = 'test-rl-parent-timeout-%s' % time.time()

        rl_dir = rospkg.RosPack().get_path('roslaunch')
        rl_file = os.path.join(rl_dir, 'resources', 'timeouts.launch')

        sigint_timeout = 2
        sigterm_timeout = 3

        p = ROSLaunchParent(run_id, [rl_file], is_core=False, port=11312, local_only=True,
                            sigint_timeout=sigint_timeout, sigterm_timeout=sigterm_timeout)
        p._load_config()
        p.pm = pmon
        p.server = ROSLaunchParentNode(p.config, pmon)

        signal_log_file = os.path.join(tempfile.gettempdir(), "signal.log")
        try:
            os.remove(signal_log_file)
        except OSError:
            pass

        def kill_launch(times):
            time.sleep(1)  # give it time to start

            times.append(time.time())
            p.shutdown()
            times.append(time.time())

        p.start()

        times = []
        t = Thread(target=kill_launch, args=(times,))
        t.start()

        p.spin()
        t.join()

        before_stop_call_time, after_stop_call_time = times

        signals = dict()
        with open(signal_log_file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                sig, timestamp = line.split(" ")
                sig = int(sig)
                timestamp = float(timestamp)
                signals[sig] = timestamp

        self.assertSetEqual({signal.SIGINT, signal.SIGTERM}, set(signals.keys()))
        self.assertAlmostEqual(before_stop_call_time, signals[signal.SIGINT], places=0)
        self.assertAlmostEqual(before_stop_call_time, signals[signal.SIGTERM] - sigint_timeout, places=0)
        self.assertAlmostEqual(before_stop_call_time, after_stop_call_time - sigint_timeout - sigterm_timeout, places=0)


def kill_parent(p, delay=1.0):
    # delay execution so that whatever pmon method we're calling has time to enter
    time.sleep(delay)
    print("stopping parent")
    p.shutdown()

