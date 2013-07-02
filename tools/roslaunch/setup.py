#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roslaunch'],
    package_dir={'': 'src'},
    scripts=['scripts/roscore',
             'scripts/roslaunch',
             'scripts/roslaunch-deps',
             'scripts/roslaunch-logs',
             'scripts/roslaunch-complete'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg']
)

setup(**d)
