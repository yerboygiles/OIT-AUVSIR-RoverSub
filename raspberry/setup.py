from setuptools import setup, Extension
from setuptools import setup, find_packages

import Cython.Build
from Cython.Build import cythonize

# Cython
ext = Extension(name="arduino_commander", sources=["arduino_commander.py"])
setup(ext_modules=cythonize(ext))

ext = Extension(name="imu_ard", sources=["imu_ard.py", "imu.py"])
setup(ext_modules=cythonize(ext))

ext = Extension(name="vector_commander", sources=["vector_commander", "imu.py"])
setup(ext_modules=cythonize(ext))
# setup(
#     name='-ANDY-',
#     version='0.1.0',
#     author='Theodor Giles',
#     author_email='theodor.giles@oit.edu',
#     url='https://github.com/yerboygiles/OIT-AUVSIR-RoverSub',
#     packages=find_packages(include=['vector_commander',
#                                     'arduino_commander',
#                                     'imu',
#                                     'imu_ard',
#                                     'task_io_lib_v2',
#                                     'vision_v1']),
#     install_requires=[
#         'pyserial>=3.5',
#         'bluerobotics-ping>=0.1.4',
#         'opencv-python>-4.5.5.64',
#         'numpy>=1.22.4'
#     ],
#     entry_points={'console_scripts': ['my-command=START_SUB:main']}
# )
