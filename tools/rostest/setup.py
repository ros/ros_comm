from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rostest'],
    package_dir={'': 'src'},
    scripts=['scripts/rostest'],
    requires=['rospkg', 'genmsg', 'genpy', 'roslib', 'rospy']
)

setup(**d)
