from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosmaster'],
    package_dir={'': 'src'},
    scripts=['scripts/rosmaster'],
    requires=['roslib', 'rospkg']
)

setup(**d)
