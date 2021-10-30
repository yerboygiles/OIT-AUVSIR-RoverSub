import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.StepperControlMode import StepperControlMode
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class Stepper(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()
		self._setTargetPosition_async = None
		self._onsetTargetPosition_async = None

		if sys.platform == 'win32':
			self._PositionChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._PositionChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._PositionChange = None
		self._onPositionChange = None

		if sys.platform == 'win32':
			self._StoppedFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		else:
			self._StoppedFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		self._Stopped = None
		self._onStopped = None

		if sys.platform == 'win32':
			self._VelocityChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._VelocityChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._VelocityChange = None
		self._onVelocityChange = None

		__func = PhidgetSupport.getDll().PhidgetStepper_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

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
			__func = PhidgetSupport.getDll().PhidgetStepper_setOnPositionChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onPositionChange, None)
		except RuntimeError:
			self._PositionChange = None
			self._onPositionChange = None

	def _localStoppedEvent(self, handle, userPtr):
		if self._Stopped == None:
			return
		self._Stopped(self)

	def setOnStoppedHandler(self, handler):
		if handler == None:
			self._Stopped = None
			self._onStopped = None
		else:
			self._Stopped = handler
			self._onStopped = self._StoppedFactory(self._localStoppedEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetStepper_setOnStoppedHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onStopped, None)
		except RuntimeError:
			self._Stopped = None
			self._onStopped = None

	def _localVelocityChangeEvent(self, handle, userPtr, velocity):
		if self._VelocityChange == None:
			return
		self._VelocityChange(self, velocity)

	def setOnVelocityChangeHandler(self, handler):
		if handler == None:
			self._VelocityChange = None
			self._onVelocityChange = None
		else:
			self._VelocityChange = handler
			self._onVelocityChange = self._VelocityChangeFactory(self._localVelocityChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetStepper_setOnVelocityChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onVelocityChange, None)
		except RuntimeError:
			self._VelocityChange = None
			self._onVelocityChange = None

	def getAcceleration(self):
		_Acceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Acceleration))

		if result > 0:
			raise PhidgetException(result)

		return _Acceleration.value

	def setAcceleration(self, Acceleration):
		_Acceleration = ctypes.c_double(Acceleration)

		__func = PhidgetSupport.getDll().PhidgetStepper_setAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Acceleration)

		if result > 0:
			raise PhidgetException(result)


	def getMinAcceleration(self):
		_MinAcceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMinAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return _MinAcceleration.value

	def getMaxAcceleration(self):
		_MaxAcceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMaxAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return _MaxAcceleration.value

	def getControlMode(self):
		_ControlMode = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetStepper_getControlMode
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ControlMode))

		if result > 0:
			raise PhidgetException(result)

		return _ControlMode.value

	def setControlMode(self, ControlMode):
		_ControlMode = ctypes.c_int(ControlMode)

		__func = PhidgetSupport.getDll().PhidgetStepper_setControlMode
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _ControlMode)

		if result > 0:
			raise PhidgetException(result)


	def getCurrentLimit(self):
		_CurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_CurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _CurrentLimit.value

	def setCurrentLimit(self, CurrentLimit):
		_CurrentLimit = ctypes.c_double(CurrentLimit)

		__func = PhidgetSupport.getDll().PhidgetStepper_setCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _CurrentLimit)

		if result > 0:
			raise PhidgetException(result)


	def getMinCurrentLimit(self):
		_MinCurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMinCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinCurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _MinCurrentLimit.value

	def getMaxCurrentLimit(self):
		_MaxCurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMaxCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxCurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _MaxCurrentLimit.value

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetStepper_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetStepper_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getEngaged(self):
		_Engaged = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetStepper_getEngaged
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Engaged))

		if result > 0:
			raise PhidgetException(result)

		return _Engaged.value

	def setEngaged(self, Engaged):
		_Engaged = ctypes.c_int(Engaged)

		__func = PhidgetSupport.getDll().PhidgetStepper_setEngaged
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Engaged)

		if result > 0:
			raise PhidgetException(result)


	def enableFailsafe(self, failsafeTime):
		_failsafeTime = ctypes.c_uint32(failsafeTime)

		__func = PhidgetSupport.getDll().PhidgetStepper_enableFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _failsafeTime)

		if result > 0:
			raise PhidgetException(result)


	def getMinFailsafeTime(self):
		_MinFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMinFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MinFailsafeTime.value

	def getMaxFailsafeTime(self):
		_MaxFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMaxFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MaxFailsafeTime.value

	def getHoldingCurrentLimit(self):
		_HoldingCurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getHoldingCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_HoldingCurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _HoldingCurrentLimit.value

	def setHoldingCurrentLimit(self, HoldingCurrentLimit):
		_HoldingCurrentLimit = ctypes.c_double(HoldingCurrentLimit)

		__func = PhidgetSupport.getDll().PhidgetStepper_setHoldingCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _HoldingCurrentLimit)

		if result > 0:
			raise PhidgetException(result)


	def getIsMoving(self):
		_IsMoving = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetStepper_getIsMoving
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_IsMoving))

		if result > 0:
			raise PhidgetException(result)

		return _IsMoving.value

	def getPosition(self):
		_Position = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Position))

		if result > 0:
			raise PhidgetException(result)

		return _Position.value

	def getMinPosition(self):
		_MinPosition = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMinPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinPosition))

		if result > 0:
			raise PhidgetException(result)

		return _MinPosition.value

	def getMaxPosition(self):
		_MaxPosition = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMaxPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxPosition))

		if result > 0:
			raise PhidgetException(result)

		return _MaxPosition.value

	def addPositionOffset(self, positionOffset):
		_positionOffset = ctypes.c_double(positionOffset)

		__func = PhidgetSupport.getDll().PhidgetStepper_addPositionOffset
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _positionOffset)

		if result > 0:
			raise PhidgetException(result)


	def getRescaleFactor(self):
		_RescaleFactor = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getRescaleFactor
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_RescaleFactor))

		if result > 0:
			raise PhidgetException(result)

		return _RescaleFactor.value

	def setRescaleFactor(self, RescaleFactor):
		_RescaleFactor = ctypes.c_double(RescaleFactor)

		__func = PhidgetSupport.getDll().PhidgetStepper_setRescaleFactor
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _RescaleFactor)

		if result > 0:
			raise PhidgetException(result)


	def resetFailsafe(self):
		__func = PhidgetSupport.getDll().PhidgetStepper_resetFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def getTargetPosition(self):
		_TargetPosition = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getTargetPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TargetPosition))

		if result > 0:
			raise PhidgetException(result)

		return _TargetPosition.value

	def setTargetPosition(self, TargetPosition):
		_TargetPosition = ctypes.c_double(TargetPosition)

		__func = PhidgetSupport.getDll().PhidgetStepper_setTargetPosition
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _TargetPosition)

		if result > 0:
			raise PhidgetException(result)


	def setTargetPosition_async(self, TargetPosition, asyncHandler):
		_TargetPosition = ctypes.c_double(TargetPosition)

		_ctx = ctypes.c_void_p()
		if asyncHandler != None:
			_ctx = ctypes.c_void_p(AsyncSupport.add(asyncHandler, self))
		_asyncHandler = AsyncSupport.getCallback()

		__func = PhidgetSupport.getDll().PhidgetStepper_setTargetPosition_async
		__func(self.handle, _TargetPosition, _asyncHandler, _ctx)


	def getVelocity(self):
		_Velocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Velocity))

		if result > 0:
			raise PhidgetException(result)

		return _Velocity.value

	def getVelocityLimit(self):
		_VelocityLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getVelocityLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_VelocityLimit))

		if result > 0:
			raise PhidgetException(result)

		return _VelocityLimit.value

	def setVelocityLimit(self, VelocityLimit):
		_VelocityLimit = ctypes.c_double(VelocityLimit)

		__func = PhidgetSupport.getDll().PhidgetStepper_setVelocityLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _VelocityLimit)

		if result > 0:
			raise PhidgetException(result)


	def getMinVelocityLimit(self):
		_MinVelocityLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMinVelocityLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinVelocityLimit))

		if result > 0:
			raise PhidgetException(result)

		return _MinVelocityLimit.value

	def getMaxVelocityLimit(self):
		_MaxVelocityLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetStepper_getMaxVelocityLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxVelocityLimit))

		if result > 0:
			raise PhidgetException(result)

		return _MaxVelocityLimit.value
