import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class BLDCMotor(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()
		self._setTargetVelocity_async = None
		self._onsetTargetVelocity_async = None

		if sys.platform == 'win32':
			self._BrakingStrengthChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._BrakingStrengthChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._BrakingStrengthChange = None
		self._onBrakingStrengthChange = None

		if sys.platform == 'win32':
			self._PositionChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._PositionChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._PositionChange = None
		self._onPositionChange = None

		if sys.platform == 'win32':
			self._VelocityUpdateFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._VelocityUpdateFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._VelocityUpdate = None
		self._onVelocityUpdate = None

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localBrakingStrengthChangeEvent(self, handle, userPtr, brakingStrength):
		if self._BrakingStrengthChange == None:
			return
		self._BrakingStrengthChange(self, brakingStrength)

	def setOnBrakingStrengthChangeHandler(self, handler):
		if handler == None:
			self._BrakingStrengthChange = None
			self._onBrakingStrengthChange = None
		else:
			self._BrakingStrengthChange = handler
			self._onBrakingStrengthChange = self._BrakingStrengthChangeFactory(self._localBrakingStrengthChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setOnBrakingStrengthChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onBrakingStrengthChange, None)
		except RuntimeError:
			self._BrakingStrengthChange = None
			self._onBrakingStrengthChange = None

	def _localPositionChangeEvent(self, handle, userPtr, position):
		if self._PositionChange == None:
			return
		self._PositionChange(self, position)

	def setOnPositionChangeHandler(self, handler):
		if handler == None:
			self._PositionChange = None
			self._onPositionChange = None
		else:
			self._PositionChange = handler
			self._onPositionChange = self._PositionChangeFactory(self._localPositionChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setOnPositionChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onPositionChange, None)
		except RuntimeError:
			self._PositionChange = None
			self._onPositionChange = None

	def _localVelocityUpdateEvent(self, handle, userPtr, velocity):
		if self._VelocityUpdate == None:
			return
		self._VelocityUpdate(self, velocity)

	def setOnVelocityUpdateHandler(self, handler):
		if handler == None:
			self._VelocityUpdate = None
			self._onVelocityUpdate = None
		else:
			self._VelocityUpdate = handler
			self._onVelocityUpdate = self._VelocityUpdateFactory(self._localVelocityUpdateEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setOnVelocityUpdateHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onVelocityUpdate, None)
		except RuntimeError:
			self._VelocityUpdate = None
			self._onVelocityUpdate = None

	def getAcceleration(self):
		_Acceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Acceleration))

		if result > 0:
			raise PhidgetException(result)

		return _Acceleration.value

	def setAcceleration(self, Acceleration):
		_Acceleration = ctypes.c_double(Acceleration)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Acceleration)

		if result > 0:
			raise PhidgetException(result)


	def getMinAcceleration(self):
		_MinAcceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return _MinAcceleration.value

	def getMaxAcceleration(self):
		_MaxAcceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return _MaxAcceleration.value

	def getBrakingStrength(self):
		_BrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_BrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _BrakingStrength.value

	def getMinBrakingStrength(self):
		_MinBrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinBrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _MinBrakingStrength.value

	def getMaxBrakingStrength(self):
		_MaxBrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxBrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _MaxBrakingStrength.value

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def enableFailsafe(self, failsafeTime):
		_failsafeTime = ctypes.c_uint32(failsafeTime)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_enableFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _failsafeTime)

		if result > 0:
			raise PhidgetException(result)


	def getMinFailsafeTime(self):
		_MinFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MinFailsafeTime.value

	def getMaxFailsafeTime(self):
		_MaxFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MaxFailsafeTime.value

	def getPosition(self):
		_Position = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Position))

		if result > 0:
			raise PhidgetException(result)

		return _Position.value

	def getMinPosition(self):
		_MinPosition = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinPosition))

		if result > 0:
			raise PhidgetException(result)

		return _MinPosition.value

	def getMaxPosition(self):
		_MaxPosition = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxPosition))

		if result > 0:
			raise PhidgetException(result)

		return _MaxPosition.value

	def addPositionOffset(self, positionOffset):
		_positionOffset = ctypes.c_double(positionOffset)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_addPositionOffset
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _positionOffset)

		if result > 0:
			raise PhidgetException(result)


	def getRescaleFactor(self):
		_RescaleFactor = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getRescaleFactor
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_RescaleFactor))

		if result > 0:
			raise PhidgetException(result)

		return _RescaleFactor.value

	def setRescaleFactor(self, RescaleFactor):
		_RescaleFactor = ctypes.c_double(RescaleFactor)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setRescaleFactor
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _RescaleFactor)

		if result > 0:
			raise PhidgetException(result)


	def resetFailsafe(self):
		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_resetFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def getStallVelocity(self):
		_StallVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getStallVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_StallVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _StallVelocity.value

	def setStallVelocity(self, StallVelocity):
		_StallVelocity = ctypes.c_double(StallVelocity)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setStallVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _StallVelocity)

		if result > 0:
			raise PhidgetException(result)


	def getMinStallVelocity(self):
		_MinStallVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinStallVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinStallVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _MinStallVelocity.value

	def getMaxStallVelocity(self):
		_MaxStallVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxStallVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxStallVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _MaxStallVelocity.value

	def getTargetBrakingStrength(self):
		_TargetBrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getTargetBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TargetBrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _TargetBrakingStrength.value

	def setTargetBrakingStrength(self, TargetBrakingStrength):
		_TargetBrakingStrength = ctypes.c_double(TargetBrakingStrength)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setTargetBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _TargetBrakingStrength)

		if result > 0:
			raise PhidgetException(result)


	def getTargetVelocity(self):
		_TargetVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getTargetVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TargetVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _TargetVelocity.value

	def setTargetVelocity(self, TargetVelocity):
		_TargetVelocity = ctypes.c_double(TargetVelocity)

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setTargetVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _TargetVelocity)

		if result > 0:
			raise PhidgetException(result)


	def setTargetVelocity_async(self, TargetVelocity, asyncHandler):
		_TargetVelocity = ctypes.c_double(TargetVelocity)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_setTargetVelocity_async
		__func(self.handle, _TargetVelocity, _asyncHandler, _ctx)


	def getVelocity(self):
		_Velocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Velocity))

		if result > 0:
			raise PhidgetException(result)

		return _Velocity.value

	def getMinVelocity(self):
		_MinVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMinVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _MinVelocity.value

	def getMaxVelocity(self):
		_MaxVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetBLDCMotor_getMaxVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _MaxVelocity.value
