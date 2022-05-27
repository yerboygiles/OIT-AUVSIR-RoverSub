from setuptools import setup, Extension

import Cython.Build
from Cython.Build import cythonize

setup(
    name='-ANDY-',
    ext_modules=cythonize("vector_commander.py")
)
