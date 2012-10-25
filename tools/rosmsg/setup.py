#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['rosmsg']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts/rosmsg', 'scripts/rosmsg-proto', 'scripts/rossrv']
d['install_requires'] = ['genmsg', 'rosbag', 'roslib', 'rospkg']

setup(**d)
