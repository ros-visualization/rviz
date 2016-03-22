#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rviz'],
    package_dir={'': 'src/python_bindings'},
    requires=['roslib', 'rospkg', 'python_qt_bindings']
)

setup(**d)
