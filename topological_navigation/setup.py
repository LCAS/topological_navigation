#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   #  don't do this unless you want a globally visible script
   # scripts=['bin/myscript'], 
   packages=['topological_navigation'],
   package_dir={'': 'src'},
   install_requires=['sympy>=1.5.1']
)

setup(**d)
