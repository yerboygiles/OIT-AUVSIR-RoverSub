import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class Accelerometer(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._AccelerationChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		else:
			self._AccelerationChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		self._AccelerationChange = None
		self._onAccelerationChange = None

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localAccelerationChangeEvent(self, handle, userPtr, acceleration, timestamp):
		if self._AccelerationChange == None:
			return
		acceleration = [acceleration[i] for i in range(3)]
		self._AccelerationChange(self, acceleration, timestamp)

	def setOnAccelerationChangeHandler(self, handler):
		if handler == None:
			self._AccelerationChange = None
			self._onAccelerationChange = None
		else:
			self._AccelerationChange = handler
			self._onAccelerationChange = self._AccelerationChangeFactory(self._localAccelerationChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetAccelerometer_setOnAccelerationChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onAccelerationChange, None)
		except RuntimeError:
			self._AccelerationChange = None
			self._onAccelerationChange = None

	def getAcceleration(self):
		_Acceleration = (ctypes.c_double * 3)()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Acceleration))

		if result > 0:
			raise PhidgetException(result)

		return list(_Acceleration)

	def getMinAcceleration(self):
		_MinAcceleration = (ctypes.c_double * 3)()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getMinAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return list(_MinAcceleration)

	def getMaxAcceleration(self):
		_MaxAcceleration = (ctypes.c_double * 3)()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getMaxAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return list(_MaxAcceleration)

	def getAccelerationChangeTrigger(self):
		_AccelerationChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getAccelerationChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_AccelerationChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _AccelerationChangeTrigger.value

	def setAccelerationChangeTrigger(self, AccelerationChangeTrigger):
		_AccelerationChangeTrigger = ctypes.c_double(AccelerationChangeTrigger)

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_setAccelerationChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _AccelerationChangeTrigger)

		if result > 0:
			raise PhidgetException(result)


	def getMinAccelerationChangeTrigger(self):
		_MinAccelerationChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getMinAccelerationChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinAccelerationChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MinAccelerationChangeTrigger.value

	def getMaxAccelerationChangeTrigger(self):
		_MaxAccelerationChangeTrigger = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getMaxAccelerationChangeTrigger
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxAccelerationChangeTrigger))

		if result > 0:
			raise PhidgetException(result)

		return _MaxAccelerationChangeTrigger.value

	def getAxisCount(self):
		_AxisCount = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getAxisCount
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_AxisCount))

		if result > 0:
			raise PhidgetException(result)

		return _AxisCount.value

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getHeatingEnabled(self):
		_HeatingEnabled = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getHeatingEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_HeatingEnabled))

		if result > 0:
			raise PhidgetException(result)

		return _HeatingEnabled.value

	def setHeatingEnabled(self, HeatingEnabled):
		_HeatingEnabled = ctypes.c_int(HeatingEnabled)

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_setHeatingEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _HeatingEnabled)

		if result > 0:
			raise PhidgetException(result)


	def getTimestamp(self):
		_Timestamp = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetAccelerometer_getTimestamp
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Timestamp))

		if result > 0:
			raise PhidgetException(result)

		return _Timestamp.value
