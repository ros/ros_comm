from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roslz4'],
    package_dir={'': 'src'},
    requires=[],
)

setup(**d)
