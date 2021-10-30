import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class CapacitiveTouch(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._TouchFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._TouchFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._Touch = None
		self._onTouch = None

		if sys.platform == 'win32':
			self._TouchEndFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		else:
			self._TouchEndFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		self._TouchEnd = None
		self._onTouchEnd = None

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localTouchEvent(self, handle, userPtr, touchValue):
		if self._Touch == None:
			return
		self._Touch(self, touchValue)

	def setOnTouchHandler(self, handler):
		if handler == None:
			self._Touch = None
			self._onTouch = None
		else:
			self._Touch = handler
			self._onTouch = self._TouchFactory(self._localTouchEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_setOnTouchHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onTouch, None)
		except RuntimeError:
			self._Touch = None
			self._onTouch = None

	def _localTouchEndEvent(self, handle, userPtr):
		if self._TouchEnd == None:
			return
		self._TouchEnd(self)

	def setOnTouchEndHandler(self, handler):
		if handler == None:
			self._TouchEnd = None
			self._onTouchEnd = None
		else:
			self._TouchEnd = handler
			self._onTouchEnd = self._TouchEndFactory(self._localTouchEndEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_setOnTouchEndHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onTouchEnd, None)
		except RuntimeError:
			self._TouchEnd = None
			self._onTouchEnd = None

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getIsTouched(self):
		_IsTouched = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getIsTouched
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_IsTouched))

		if result > 0:
			raise PhidgetException(result)

		return _IsTouched.value

	def getSensitivity(self):
		_Sensitivity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getSensitivity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Sensitivity))

		if result > 0:
			raise PhidgetException(result)

		return _Sensitivity.value

	def setSensitivity(self, Sensitivity):
		_Sensitivity = ctypes.c_double(Sensitivity)

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_setSensitivity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Sensitivity)

		if result > 0:
			raise PhidgetException(result)


	def getMinSensitivity(self):
		_MinSensitivity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMinSensitivity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinSensitivity))

		if result > 0:
			raise PhidgetException(result)

		return _MinSensitivity.value

	def getMaxSensitivity(self):
		_MaxSensitivity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMaxSensitivity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxSensitivity))

		if result > 0:
			raise PhidgetException(result)

		return _MaxSensitivity.value

	def getTouchValue(self):
		_TouchValue = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getTouchValue
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TouchValue))

		if result > 0:
			raise PhidgetException(result)

		return _TouchValue.value

	def getMinTouchValue(self):
		_MinTouchValue = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMinTouchValue
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinTouchValue))

		if result > 0:
			raise PhidgetException(result)

		return _MinTouchValue.value

	def getMaxTouchValue(self):
		_MaxTouchValue = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMaxTouchValue
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxTouchValue))

		if result > 0:
			raise PhidgetException(result)

		return _MaxTouchValue.value

	def getTouchValueChangeTrigger(self):
		_TouchValueChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getTouchValueChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TouchValueChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _TouchValueChangeTrigger.value

	def setTouchValueChangeTrigger(self, TouchValueChangeTrigger):
		_TouchValueChangeTrigger = ctypes.c_double(TouchValueChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_setTouchValueChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _TouchValueChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getMinTouchValueChangeTrigger(self):
		_MinTouchValueChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMinTouchValueChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinTouchValueChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MinTouchValueChangeTrigger.value

	def getMaxTouchValueChangeTrigger(self):
		_MaxTouchValueChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCapacitiveTouch_getMaxTouchValueChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxTouchValueChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MaxTouchValueChangeTrigger.value
