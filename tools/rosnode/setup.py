from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosnode'],
    package_dir={'': 'src'},
    scripts=['scripts/rosnode'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg']
)

setup(**d)
