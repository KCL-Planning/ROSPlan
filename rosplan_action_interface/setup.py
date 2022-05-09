#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
  packages=['rosplan_action_interface'],
  package_dir={'': 'src'}
)

setup(**d)
