from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosbag'],
    package_dir={'': 'src'},
    scripts=['scripts/rosbag'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg']
)

setup(**d)
