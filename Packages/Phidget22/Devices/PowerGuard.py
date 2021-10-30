import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.FanMode import FanMode
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class PowerGuard(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def enableFailsafe(self, failsafeTime):
		_failsafeTime = ctypes.c_uint32(failsafeTime)

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_enableFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _failsafeTime)

		if result > 0:
			raise PhidgetException(result)


	def getMinFailsafeTime(self):
		_MinFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getMinFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MinFailsafeTime.value

	def getMaxFailsafeTime(self):
		_MaxFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getMaxFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MaxFailsafeTime.value

	def getFanMode(self):
		_FanMode = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getFanMode
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_FanMode))

		if result > 0:
			raise PhidgetException(result)

		return _FanMode.value

	def setFanMode(self, FanMode):
		_FanMode = ctypes.c_int(FanMode)

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_setFanMode
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _FanMode)

		if result > 0:
			raise PhidgetException(result)


	def getOverVoltage(self):
		_OverVoltage = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getOverVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_OverVoltage))

		if result > 0:
			raise PhidgetException(result)

		return _OverVoltage.value

	def setOverVoltage(self, OverVoltage):
		_OverVoltage = ctypes.c_double(OverVoltage)

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_setOverVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _OverVoltage)

		if result > 0:
			raise PhidgetException(result)


	def getMinOverVoltage(self):
		_MinOverVoltage = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getMinOverVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinOverVoltage))

		if result > 0:
			raise PhidgetException(result)

		return _MinOverVoltage.value

	def getMaxOverVoltage(self):
		_MaxOverVoltage = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getMaxOverVoltage
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxOverVoltage))

		if result > 0:
			raise PhidgetException(result)

		return _MaxOverVoltage.value

	def getPowerEnabled(self):
		_PowerEnabled = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_getPowerEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_PowerEnabled))

		if result > 0:
			raise PhidgetException(result)

		return _PowerEnabled.value

	def setPowerEnabled(self, PowerEnabled):
		_PowerEnabled = ctypes.c_int(PowerEnabled)

		__func = PhidgetSupport.getDll().PhidgetPowerGuard_setPowerEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _PowerEnabled)

		if result > 0:
			raise PhidgetException(result)


	def resetFailsafe(self):
		__func = PhidgetSupport.getDll().PhidgetPowerGuard_resetFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)

