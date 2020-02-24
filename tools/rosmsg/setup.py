from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosmsg'],
    package_dir={'': 'src'},
    scripts=['scripts/rosmsg', 'scripts/rosmsg-proto', 'scripts/rossrv'],
    requires=['genmsg', 'rosbag', 'roslib', 'rospkg']
)

setup(**d)
