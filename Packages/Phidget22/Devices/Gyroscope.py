import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class Gyroscope(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._AngularRateUpdateFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		else:
			self._AngularRateUpdateFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		self._AngularRateUpdate = None
		self._onAngularRateUpdate = None

		__func = PhidgetSupport.getDll().PhidgetGyroscope_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localAngularRateUpdateEvent(self, handle, userPtr, angularRate, timestamp):
		if self._AngularRateUpdate == None:
			return
		angularRate = [angularRate[i] for i in range(3)]
		self._AngularRateUpdate(self, angularRate, timestamp)

	def setOnAngularRateUpdateHandler(self, handler):
		if handler == None:
			self._AngularRateUpdate = None
			self._onAngularRateUpdate = None
		else:
			self._AngularRateUpdate = handler
			self._onAngularRateUpdate = self._AngularRateUpdateFactory(self._localAngularRateUpdateEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetGyroscope_setOnAngularRateUpdateHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onAngularRateUpdate, None)
		except RuntimeError:
			self._AngularRateUpdate = None
			self._onAngularRateUpdate = None

	def getAngularRate(self):
		_AngularRate = (ctypes.c_double * 3)()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getAngularRate
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_AngularRate))

		if result > 0:
			raise PhidgetException(result)

		return list(_AngularRate)

	def getMinAngularRate(self):
		_MinAngularRate = (ctypes.c_double * 3)()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getMinAngularRate
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinAngularRate))

		if result > 0:
			raise PhidgetException(result)

		return list(_MinAngularRate)

	def getMaxAngularRate(self):
		_MaxAngularRate = (ctypes.c_double * 3)()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getMaxAngularRate
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxAngularRate))

		if result > 0:
			raise PhidgetException(result)

		return list(_MaxAngularRate)

	def getAxisCount(self):
		_AxisCount = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getAxisCount
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_AxisCount))

		if result > 0:
			raise PhidgetException(result)

		return _AxisCount.value

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetGyroscope_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getHeatingEnabled(self):
		_HeatingEnabled = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getHeatingEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_HeatingEnabled))

		if result > 0:
			raise PhidgetException(result)

		return _HeatingEnabled.value

	def setHeatingEnabled(self, HeatingEnabled):
		_HeatingEnabled = ctypes.c_int(HeatingEnabled)

		__func = PhidgetSupport.getDll().PhidgetGyroscope_setHeatingEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _HeatingEnabled)

		if result > 0:
			raise PhidgetException(result)


	def getTimestamp(self):
		_Timestamp = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetGyroscope_getTimestamp
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Timestamp))

		if result > 0:
			raise PhidgetException(result)

		return _Timestamp.value

	def zero(self):
		__func = PhidgetSupport.getDll().PhidgetGyroscope_zero
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)

