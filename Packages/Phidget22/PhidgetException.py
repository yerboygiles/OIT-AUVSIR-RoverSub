import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.ErrorCode import ErrorCode

class PhidgetException(Exception):

	def __init__(self,code):

		_code = ctypes.c_int()
		_desc = ctypes.c_char_p()
		_detailLen = ctypes.c_size_t()

		result = PhidgetSupport.getDll().Phidget_getLastError(ctypes.byref(_code), ctypes.byref(_desc), None, ctypes.byref(_detailLen))
		if result == 0 and _code.value == code:
			_detail = ctypes.create_string_buffer(_detailLen.value)
			result = PhidgetSupport.getDll().Phidget_getLastError(ctypes.byref(_code), ctypes.byref(_desc), _detail, ctypes.byref(_detailLen))
			if result == 0:
				self.code = _code.value
				self.details = _detail.value.decode("utf-8");
				self.description = _desc.value.decode("utf-8");
				return

		result = PhidgetSupport.getDll().Phidget_getErrorDescription(_code, ctypes.byref(_desc))

		if result == 0:
			self.code = code
			self.details = ""
			self.description = _desc.value.decode("utf-8");
			return

		self.code = code
		self.details = ""
		self.description = "";

	def __str__(self):
		return "PhidgetException 0x%02x (%s)\n%s" % (self.code, self.description, self.details)

	def __del__(self):
		pass
