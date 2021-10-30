import threading
import sys
import ctypes
from ctypes import *
import os

class PhidgetSupport:
	__dll = None

	@staticmethod
	def getDll():
		if PhidgetSupport.__dll is None:
			if sys.platform == 'win32':
				# Load phidget22.dll from within the Python package itself
				libs_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".libs")
				if os.path.exists(os.path.join(libs_path, "phidget22.dll")):
					PhidgetSupport.__dll = windll.LoadLibrary(os.path.join(libs_path, "phidget22.dll"))
				else:
					PhidgetSupport.__dll = windll.LoadLibrary("phidget22.dll")
			elif sys.platform == 'darwin':
				PhidgetSupport.__dll = cdll.LoadLibrary("/Library/Frameworks/Phidget22.framework/Versions/Current/Phidget22")
			else:
				PhidgetSupport.__dll = cdll.LoadLibrary("libphidget22.so.0")
		return PhidgetSupport.__dll

	def __init__(self):
		self.handle = None

	def __del__(self):
		pass

	@staticmethod
	def versionChecked_ord(character):
		if(sys.version_info[0] < 3):
			return character
		else:
			return ord(character)
