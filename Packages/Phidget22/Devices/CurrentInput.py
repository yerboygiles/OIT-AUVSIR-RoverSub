import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.PowerSupply import PowerSupply
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class CurrentInput(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._CurrentChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._CurrentChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._CurrentChange = None
		self._onCurrentChange = None

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localCurrentChangeEvent(self, handle, userPtr, current):
		if self._CurrentChange == None:
			return
		self._CurrentChange(self, current)

	def setOnCurrentChangeHandler(self, handler):
		if handler == None:
			self._CurrentChange = None
			self._onCurrentChange = None
		else:
			self._CurrentChange = handler
			self._onCurrentChange = self._CurrentChangeFactory(self._localCurrentChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetCurrentInput_setOnCurrentChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onCurrentChange, None)
		except RuntimeError:
			self._CurrentChange = None
			self._onCurrentChange = None

	def getCurrent(self):
		_Current = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getCurrent
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Current))

		if result > 0:
			raise PhidgetException(result)

		return _Current.value

	def getMinCurrent(self):
		_MinCurrent = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getMinCurrent
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinCurrent))

		if result > 0:
			raise PhidgetException(result)

		return _MinCurrent.value

	def getMaxCurrent(self):
		_MaxCurrent = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getMaxCurrent
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxCurrent))

		if result > 0:
			raise PhidgetException(result)

		return _MaxCurrent.value

	def getCurrentChangeTrigger(self):
		_CurrentChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getCurrentChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_CurrentChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _CurrentChangeTrigger.value

	def setCurrentChangeTrigger(self, CurrentChangeTrigger):
		_CurrentChangeTrigger = ctypes.c_double(CurrentChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_setCurrentChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _CurrentChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getMinCurrentChangeTrigger(self):
		_MinCurrentChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getMinCurrentChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinCurrentChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MinCurrentChangeTrigger.value

	def getMaxCurrentChangeTrigger(self):
		_MaxCurrentChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getMaxCurrentChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxCurrentChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MaxCurrentChangeTrigger.value

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getPowerSupply(self):
		_PowerSupply = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_getPowerSupply
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_PowerSupply))

		if result > 0:
			raise PhidgetException(result)

		return _PowerSupply.value

	def setPowerSupply(self, PowerSupply):
		_PowerSupply = ctypes.c_int(PowerSupply)

		__func = PhidgetSupport.getDll().PhidgetCurrentInput_setPowerSupply
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _PowerSupply)

		if result > 0:
			raise PhidgetException(result)

