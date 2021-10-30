import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.RTDWireSetup import RTDWireSetup
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class ResistanceInput(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._ResistanceChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._ResistanceChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._ResistanceChange = None
		self._onResistanceChange = None

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localResistanceChangeEvent(self, handle, userPtr, resistance):
		if self._ResistanceChange == None:
			return
		self._ResistanceChange(self, resistance)

	def setOnResistanceChangeHandler(self, handler):
		if handler == None:
			self._ResistanceChange = None
			self._onResistanceChange = None
		else:
			self._ResistanceChange = handler
			self._onResistanceChange = self._ResistanceChangeFactory(self._localResistanceChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetResistanceInput_setOnResistanceChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onResistanceChange, None)
		except RuntimeError:
			self._ResistanceChange = None
			self._onResistanceChange = None

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getResistance(self):
		_Resistance = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getResistance
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Resistance))

		if result > 0:
			raise PhidgetException(result)

		return _Resistance.value

	def getMinResistance(self):
		_MinResistance = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getMinResistance
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinResistance))

		if result > 0:
			raise PhidgetException(result)

		return _MinResistance.value

	def getMaxResistance(self):
		_MaxResistance = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getMaxResistance
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxResistance))

		if result > 0:
			raise PhidgetException(result)

		return _MaxResistance.value

	def getResistanceChangeTrigger(self):
		_ResistanceChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getResistanceChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ResistanceChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _ResistanceChangeTrigger.value

	def setResistanceChangeTrigger(self, ResistanceChangeTrigger):
		_ResistanceChangeTrigger = ctypes.c_double(ResistanceChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_setResistanceChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _ResistanceChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getMinResistanceChangeTrigger(self):
		_MinResistanceChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getMinResistanceChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinResistanceChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MinResistanceChangeTrigger.value

	def getMaxResistanceChangeTrigger(self):
		_MaxResistanceChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getMaxResistanceChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxResistanceChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MaxResistanceChangeTrigger.value

	def getRTDWireSetup(self):
		_RTDWireSetup = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_getRTDWireSetup
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_RTDWireSetup))

		if result > 0:
			raise PhidgetException(result)

		return _RTDWireSetup.value

	def setRTDWireSetup(self, RTDWireSetup):
		_RTDWireSetup = ctypes.c_int(RTDWireSetup)

		__func = PhidgetSupport.getDll().PhidgetResistanceInput_setRTDWireSetup
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _RTDWireSetup)

		if result > 0:
			raise PhidgetException(result)

