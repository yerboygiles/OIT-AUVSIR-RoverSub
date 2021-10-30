import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.RTDType import RTDType
from Phidget22.RTDWireSetup import RTDWireSetup
from Phidget22.ThermocoupleType import ThermocoupleType
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class TemperatureSensor(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._TemperatureChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._TemperatureChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._TemperatureChange = None
		self._onTemperatureChange = None

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localTemperatureChangeEvent(self, handle, userPtr, temperature):
		if self._TemperatureChange == None:
			return
		self._TemperatureChange(self, temperature)

	def setOnTemperatureChangeHandler(self, handler):
		if handler == None:
			self._TemperatureChange = None
			self._onTemperatureChange = None
		else:
			self._TemperatureChange = handler
			self._onTemperatureChange = self._TemperatureChangeFactory(self._localTemperatureChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_setOnTemperatureChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onTemperatureChange, None)
		except RuntimeError:
			self._TemperatureChange = None
			self._onTemperatureChange = None

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getRTDType(self):
		_RTDType = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getRTDType
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_RTDType))

		if result > 0:
			raise PhidgetException(result)

		return _RTDType.value

	def setRTDType(self, RTDType):
		_RTDType = ctypes.c_int(RTDType)

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_setRTDType
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _RTDType)

		if result > 0:
			raise PhidgetException(result)


	def getRTDWireSetup(self):
		_RTDWireSetup = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getRTDWireSetup
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_RTDWireSetup))

		if result > 0:
			raise PhidgetException(result)

		return _RTDWireSetup.value

	def setRTDWireSetup(self, RTDWireSetup):
		_RTDWireSetup = ctypes.c_int(RTDWireSetup)

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_setRTDWireSetup
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _RTDWireSetup)

		if result > 0:
			raise PhidgetException(result)


	def getTemperature(self):
		_Temperature = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getTemperature
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Temperature))

		if result > 0:
			raise PhidgetException(result)

		return _Temperature.value

	def getMinTemperature(self):
		_MinTemperature = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getMinTemperature
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinTemperature))

		if result > 0:
			raise PhidgetException(result)

		return _MinTemperature.value

	def getMaxTemperature(self):
		_MaxTemperature = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getMaxTemperature
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxTemperature))

		if result > 0:
			raise PhidgetException(result)

		return _MaxTemperature.value

	def getTemperatureChangeTrigger(self):
		_TemperatureChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getTemperatureChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TemperatureChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _TemperatureChangeTrigger.value

	def setTemperatureChangeTrigger(self, TemperatureChangeTrigger):
		_TemperatureChangeTrigger = ctypes.c_double(TemperatureChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_setTemperatureChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _TemperatureChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getMinTemperatureChangeTrigger(self):
		_MinTemperatureChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getMinTemperatureChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinTemperatureChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MinTemperatureChangeTrigger.value

	def getMaxTemperatureChangeTrigger(self):
		_MaxTemperatureChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getMaxTemperatureChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxTemperatureChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MaxTemperatureChangeTrigger.value

	def getThermocoupleType(self):
		_ThermocoupleType = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_getThermocoupleType
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ThermocoupleType))

		if result > 0:
			raise PhidgetException(result)

		return _ThermocoupleType.value

	def setThermocoupleType(self, ThermocoupleType):
		_ThermocoupleType = ctypes.c_int(ThermocoupleType)

		__func = PhidgetSupport.getDll().PhidgetTemperatureSensor_setThermocoupleType
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _ThermocoupleType)

		if result > 0:
			raise PhidgetException(result)

