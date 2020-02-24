from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosgraph', 'rosgraph.impl'],
    package_dir={'': 'src'},
    scripts=['scripts/rosgraph'],
    requires=['rospkg']
)

setup(**d)
