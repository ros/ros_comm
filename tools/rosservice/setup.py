from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosservice'],
    package_dir={'': 'src'},
    scripts=['scripts/rosservice'],
    requires=['genpy', 'rosgraph', 'roslib', 'rospy', 'rosmsg']
)

setup(**d)
