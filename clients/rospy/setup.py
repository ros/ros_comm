from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rospy', 'rospy.impl'],
    package_dir={'': 'src'},
    scripts=['scripts/rosconsole'],
    requires=['genpy', 'numpy', 'rosgraph', 'roslib', 'rospkg']
)

setup(**d)
