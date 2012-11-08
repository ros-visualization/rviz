#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['rviz']
d['package_dir'] = {'': 'src/python_bindings'}
d['install_requires'] = ['roslib', 'rospkg', 'python_qt_bindings']

setup(**d)
