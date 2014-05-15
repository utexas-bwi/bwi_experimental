#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bwi_coffee_new'],
    package_dir={'': 'src'}
)

setup(**d)
