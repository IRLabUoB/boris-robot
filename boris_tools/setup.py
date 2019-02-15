#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['boris_tools','boris_joint_trajectory_action']
d['package_dir'] = {'': 'src'}

setup(**d)
