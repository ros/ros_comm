#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sros'],
    package_dir={'': 'src'},
    scripts=['scripts/sroscore', 'scripts/sroslaunch', 'scripts/sros'],
    requires=[]
)
#requires=['genmsg', 'genpy', 'roslib', 'rospkg']

setup(**d)
