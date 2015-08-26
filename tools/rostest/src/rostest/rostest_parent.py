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

import logging
import sys

import rosgraph
import roslaunch.config
from roslaunch.core import printlog_bold, RLException
import roslaunch.launch
import roslaunch.pmon
import roslaunch.server
import roslaunch.xmlloader 

import roslaunch.parent

from rosmaster.master import Master
from rospy import logwarn

class ROSTestLaunchParent(roslaunch.parent.ROSLaunchParent):

    def __init__(self, config, roslaunch_files, port=0, reuse_master=False, clear=False):
        if config is None:
            raise Exception("config not initialized")
        # we generate a run_id for each test
        if reuse_master:
            param_server = rosgraph.Master('/roslaunch')
            try:
                run_id = param_server.getParam('/run_id')
            except Exception as e:
                # The user asked us to connect to an existing ROS master, and
                # we can't. Throw an exception and die
                raise Exception("Could not connect to existing ROS master. "
                                + "Original exception was: %s" % str(e))
            except:
                # oh boy; we got something that wasn't an exception.
                # Throw an exception and die
                raise Exception("Could not connect to existing ROS master.")

            if clear:
                params = param_server.getParamNames()
                # whitelist of parameters to keep
                whitelist = ['/run_id', '/rosversion', '/rosdistro']
                for i in reversed(range(len(params))):
                    param = params[i]
                    if param in whitelist:
                        del params[i]
                    elif param.startswith('/roslaunch/'):
                        del params[i]
                for param in params:
                    param_server.deleteParam(param)
        else:
            run_id = roslaunch.core.generate_run_id()
        super(ROSTestLaunchParent, self).__init__(run_id, roslaunch_files, is_core=False, is_rostest=True)
        self.config = config
        self.port = port
        self.reuse_master = reuse_master
        self.master = None
        
    def _load_config(self):
        # disable super, just in case, though this shouldn't get called
        pass

    def setUp(self):
        """
        initializes self.config and xmlrpc infrastructure
        """
        self._start_infrastructure()
        if not self.reuse_master:
            self.master = Master(port=self.port)
            self.master.start()
            self.config.master.uri = self.master.uri
        self._init_runner()

    def tearDown(self):
        if self.runner is not None:
            runner = self.runner
            runner.stop()
        if self.master is not None:
            self.master.stop()
            self.master = None
        self._stop_infrastructure()

    def launch(self):
        """
        perform launch of nodes, does not launch tests.  rostest_parent
        follows a different pattern of init/run than the normal
        roslaunch, which is why it does not reuse start()/spin()
        """
        if self.runner is not None:
            return self.runner.launch()
        else:
            raise Exception("no runner to launch")

    def run_test(self, test):
        """
        run the test, blocks until completion 
        """
        if self.runner is not None:
            # run the test, blocks until completion            
            return self.runner.run_test(test)
        else:
            raise Exception("no runner")
