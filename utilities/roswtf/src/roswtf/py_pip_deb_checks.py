# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Checks to see if core Python scripts have:
1) Been installed
2) Have been installed via Debians on Ubuntu
3) Have not been installed via pip on Ubuntu
"""

from __future__ import print_function

import subprocess
import importlib
import os

#A dictionary of core ROS python packages and their corresponding .deb packages
py_to_deb_packages = {
    'bloom': 'python-bloom',
    'catkin': 'python-catkin',
    'rospkg': 'python-rospkg',
    'rosinstall': 'python-rosinstall',
    'rosrelease': 'python-rosrelease',
    'rosdep2': 'python-rosdep',
}


def get_host_os():
    """Determines the name of the host operating system"""
    import rospkg.os_detect
    os_detector = rospkg.os_detect.OsDetect()
    return (os_detector.detect_os())[0]


def is_host_os_ubuntu():
    """Indicates if the host operating system is Ubuntu"""
    return (get_host_os() == 'ubuntu')


def is_debian_package_installed(deb_pkg):
    """Uses dpkg to determine if a package has been installed"""
    return (subprocess.call(
        'dpkg -l ' + deb_pkg,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE) == 0)


def is_a_pip_path_on_ubuntu(path):
    """Indicates if a path (either directory or file) is in the same place
    pip installs Python code"""
    return ('/usr/local' in path)


def is_python_package_installed(python_pkg):
    """Indicates if a Python package is importable in the current
    environment."""
    try:
        importlib.import_module(python_pkg)
        return True
    except ImportError:
        return False


def is_python_package_installed_via_pip_on_ubuntu(python_pkg):
    """Indicates if am importable package has been installed through pip on
    Ubuntu"""
    try:
        pkg_handle = importlib.import_module(python_pkg)
        return is_a_pip_path_on_ubuntu(pkg_handle.__file__)
    except ImportError:
        return False


# Error/Warning Rules
def python_module_install_check(ctx):
    """Make sure core Python modules are installed"""
    warn_str = ''
    for py_pkg in py_to_deb_packages:
        if not is_python_package_installed(py_pkg):
            warn_str = warn_str + py_pkg + ' -- '
    if (warn_str != ''):
        return warn_str


def deb_install_check_on_ubuntu(ctx):
    """Make sure on Debian python packages are installed"""
    if (is_host_os_ubuntu()):
        warn_str = ''
        for py_pkg in py_to_deb_packages:
            deb_pkg = py_to_deb_packages[py_pkg]
            if not is_debian_package_installed(deb_pkg):
                warn_str = warn_str + py_pkg + ' (' + deb_pkg + ') -- '
        if (warn_str != ''):
            return warn_str


def pip_install_check_on_ubuntu(ctx):
    """Make sure on Ubuntu, Python packages are install with apt and not pip"""
    if (is_host_os_ubuntu()):
        warn_str = ''
        for py_pkg in py_to_deb_packages:
            if is_python_package_installed_via_pip_on_ubuntu(py_pkg):
                warn_str = warn_str + py_pkg + ' -- '
        if (warn_str != ''):
            return warn_str

warnings = [
    (python_module_install_check,
     "You are missing core ROS Python modules: "),
    (pip_install_check_on_ubuntu,
     "You have pip installed packages on Ubuntu, "
     "remove and install using Debian packages: "),
    (deb_install_check_on_ubuntu,
     "You are missing Debian packages for core ROS Python modules: "),
    ]

errors = []


def wtf_check(ctx):
    """Check implementation function for roswtf"""
    from roswtf.rules import warning_rule, error_rule
    for r in warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in errors:
        error_rule(r, r[0](ctx), ctx)
