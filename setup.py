#!/usr/bin/env python
# coding: utf-8
import os
from setuptools import setup, find_packages

try:
    import pypandoc
    long_description = pypandoc.convert('README.md', 'rst')
except:
    long_description = ''

classifiers = ['Development Status :: 4 - Beta',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: Apache Software License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 2.7',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

setup(
        name             = 'MotorShield',
        version          = __version__,
        description      = 'Motor Shield Lib',
        license          = __license__,
        author           = __author__,
        author_email     = 'akira@fabo.o',
        url              = 'https://github.com/FaBoPlatform/MotorShield/',
        keywords         = 'Motor Shield Lib',
        packages         = find_packages(),
        install_requires = [],
        )