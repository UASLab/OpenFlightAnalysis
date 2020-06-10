"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

from distutils.core import setup

setup(
    name = 'OpenFlightAnalysis',
    author = 'Chris Regan',
    packages = ['Test', 'Core'],
    license = 'LICENSE.md',
    long_description = open('README.md').read(),
)