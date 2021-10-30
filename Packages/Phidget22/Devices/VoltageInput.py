import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.PowerSupply import PowerSupply
from Phidget22.VoltageSensorType import VoltageSensorType
from Phidget22.UnitInfo import UnitInfo
from Phidget22.Unit import Unit
from Phidget22.VoltageRange import VoltageRange
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class VoltageInput(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._SensorChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double, ctypes.POINTER(UnitInfo))
		else:
			self._SensorChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double, ctypes.POINTER(UnitInfo))
		self._SensorChange = None
		self._onSensorChange = None

		if sys.platform == 'win32':
			self._VoltageChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._VoltageChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._VoltageChange = None
		self._onVoltageChange = None

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localSensorChangeEvent(self, handle, userPtr, sensorValue, sensorUnit):
		if self._SensorChange == None:
			return
		if sensorUnit != None:
			sensorUnit = sensorUnit.contents
			sensorUnit.toPython()
		self._SensorChange(self, sensorValue, sensorUnit)

	def setOnSensorChangeHandler(self, handler):
		if handler == None:
			self._SensorChange = None
			self._onSensorChange = None
		else:
			self._SensorChange = handler
			self._onSensorChange = self._SensorChangeFactory(self._localSensorChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetVoltageInput_setOnSensorChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onSensorChange, None)
		except RuntimeError:
			self._SensorChange = None
			self._onSensorChange = None

	def _localVoltageChangeEvent(self, handle, userPtr, voltage):
		if self._VoltageChange == None:
			return
		self._VoltageChange(self, voltage)

	def setOnVoltageChangeHandler(self, handler):
		if handler == None:
			self._VoltageChange = None
			self._onVoltageChange = None
		else:
			self._VoltageChange = handler
			self._onVoltageChange = self._VoltageChangeFactory(self._localVoltageChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetVoltageInput_setOnVoltageChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onVoltageChange, None)
		except RuntimeError:
			self._VoltageChange = None
			self._onVoltageChange = None

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getPowerSupply(self):
		_PowerSupply = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getPowerSupply
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_PowerSupply))

		if result > 0:
			raise PhidgetException(result)

		return _PowerSupply.value

	def setPowerSupply(self, PowerSupply):
		_PowerSupply = ctypes.c_int(PowerSupply)

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_setPowerSupply
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _PowerSupply)

		if result > 0:
			raise PhidgetException(result)


	def getSensorType(self):
		_SensorType = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getSensorType
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_SensorType))

		if result > 0:
			raise PhidgetException(result)

		return _SensorType.value

	def setSensorType(self, SensorType):
		_SensorType = ctypes.c_int(SensorType)

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_setSensorType
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _SensorType)

		if result > 0:
			raise PhidgetException(result)


	def getSensorUnit(self):
		_SensorUnit = UnitInfo()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getSensorUnit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_SensorUnit))

		if result > 0:
			raise PhidgetException(result)

		return _SensorUnit.toPython()

	def getSensorValue(self):
		_SensorValue = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getSensorValue
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_SensorValue))

		if result > 0:
			raise PhidgetException(result)

		return _SensorValue.value

	def getSensorValueChangeTrigger(self):
		_SensorValueChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getSensorValueChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_SensorValueChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _SensorValueChangeTrigger.value

	def setSensorValueChangeTrigger(self, SensorValueChangeTrigger):
		_SensorValueChangeTrigger = ctypes.c_double(SensorValueChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_setSensorValueChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _SensorValueChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getVoltage(self):
		_Voltage = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Voltage))

		if result > 0:
			raise PhidgetException(result)

		return _Voltage.value

	def getMinVoltage(self):
		_MinVoltage = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getMinVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinVoltage))

		if result > 0:
			raise PhidgetException(result)

		return _MinVoltage.value

	def getMaxVoltage(self):
		_MaxVoltage = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getMaxVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxVoltage))

		if result > 0:
			raise PhidgetException(result)

		return _MaxVoltage.value

	def getVoltageChangeTrigger(self):
		_VoltageChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getVoltageChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_VoltageChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _VoltageChangeTrigger.value

	def setVoltageChangeTrigger(self, VoltageChangeTrigger):
		_VoltageChangeTrigger = ctypes.c_double(VoltageChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_setVoltageChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _VoltageChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getMinVoltageChangeTrigger(self):
		_MinVoltageChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getMinVoltageChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinVoltageChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MinVoltageChangeTrigger.value

	def getMaxVoltageChangeTrigger(self):
		_MaxVoltageChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getMaxVoltageChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxVoltageChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MaxVoltageChangeTrigger.value

	def getVoltageRange(self):
		_VoltageRange = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_getVoltageRange
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_VoltageRange))

		if result > 0:
			raise PhidgetException(result)

		return _VoltageRange.value

	def setVoltageRange(self, VoltageRange):
		_VoltageRange = ctypes.c_int(VoltageRange)

		__func = PhidgetSupport.getDll().PhidgetVoltageInput_setVoltageRange
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _VoltageRange)

		if result > 0:
			raise PhidgetException(result)

