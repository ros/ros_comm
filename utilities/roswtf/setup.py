#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roswtf'],
    package_dir={'': 'src'},
    scripts=['scripts/roswtf'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg']
)

setup(**d)
