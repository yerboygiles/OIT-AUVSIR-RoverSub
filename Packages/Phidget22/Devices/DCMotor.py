import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.FanMode import FanMode
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class DCMotor(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()
		self._setTargetVelocity_async = None
		self._onsetTargetVelocity_async = None

		if sys.platform == 'win32':
			self._BackEMFChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._BackEMFChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._BackEMFChange = None
		self._onBackEMFChange = None

		if sys.platform == 'win32':
			self._BrakingStrengthChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._BrakingStrengthChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._BrakingStrengthChange = None
		self._onBrakingStrengthChange = None

		if sys.platform == 'win32':
			self._VelocityUpdateFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		else:
			self._VelocityUpdateFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double)
		self._VelocityUpdate = None
		self._onVelocityUpdate = None

		__func = PhidgetSupport.getDll().PhidgetDCMotor_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localBackEMFChangeEvent(self, handle, userPtr, backEMF):
		if self._BackEMFChange == None:
			return
		self._BackEMFChange(self, backEMF)

	def setOnBackEMFChangeHandler(self, handler):
		if handler == None:
			self._BackEMFChange = None
			self._onBackEMFChange = None
		else:
			self._BackEMFChange = handler
			self._onBackEMFChange = self._BackEMFChangeFactory(self._localBackEMFChangeEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetDCMotor_setOnBackEMFChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onBackEMFChange, None)
		except RuntimeError:
			self._BackEMFChange = None
			self._onBackEMFChange = None

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
			__func = PhidgetSupport.getDll().PhidgetDCMotor_setOnBrakingStrengthChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onBrakingStrengthChange, None)
		except RuntimeError:
			self._BrakingStrengthChange = None
			self._onBrakingStrengthChange = None

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
			__func = PhidgetSupport.getDll().PhidgetDCMotor_setOnVelocityUpdateHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onVelocityUpdate, None)
		except RuntimeError:
			self._VelocityUpdate = None
			self._onVelocityUpdate = None

	def getAcceleration(self):
		_Acceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Acceleration))

		if result > 0:
			raise PhidgetException(result)

		return _Acceleration.value

	def setAcceleration(self, Acceleration):
		_Acceleration = ctypes.c_double(Acceleration)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Acceleration)

		if result > 0:
			raise PhidgetException(result)


	def getMinAcceleration(self):
		_MinAcceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return _MinAcceleration.value

	def getMaxAcceleration(self):
		_MaxAcceleration = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxAcceleration
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxAcceleration))

		if result > 0:
			raise PhidgetException(result)

		return _MaxAcceleration.value

	def getBackEMF(self):
		_BackEMF = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getBackEMF
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_BackEMF))

		if result > 0:
			raise PhidgetException(result)

		return _BackEMF.value

	def getBackEMFSensingState(self):
		_BackEMFSensingState = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getBackEMFSensingState
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_BackEMFSensingState))

		if result > 0:
			raise PhidgetException(result)

		return _BackEMFSensingState.value

	def setBackEMFSensingState(self, BackEMFSensingState):
		_BackEMFSensingState = ctypes.c_int(BackEMFSensingState)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setBackEMFSensingState
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _BackEMFSensingState)

		if result > 0:
			raise PhidgetException(result)


	def getBrakingStrength(self):
		_BrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_BrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _BrakingStrength.value

	def getMinBrakingStrength(self):
		_MinBrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinBrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _MinBrakingStrength.value

	def getMaxBrakingStrength(self):
		_MaxBrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxBrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _MaxBrakingStrength.value

	def getCurrentLimit(self):
		_CurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_CurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _CurrentLimit.value

	def setCurrentLimit(self, CurrentLimit):
		_CurrentLimit = ctypes.c_double(CurrentLimit)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _CurrentLimit)

		if result > 0:
			raise PhidgetException(result)


	def getMinCurrentLimit(self):
		_MinCurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinCurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _MinCurrentLimit.value

	def getMaxCurrentLimit(self):
		_MaxCurrentLimit = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxCurrentLimit
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxCurrentLimit))

		if result > 0:
			raise PhidgetException(result)

		return _MaxCurrentLimit.value

	def getCurrentRegulatorGain(self):
		_CurrentRegulatorGain = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getCurrentRegulatorGain
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_CurrentRegulatorGain))

		if result > 0:
			raise PhidgetException(result)

		return _CurrentRegulatorGain.value

	def setCurrentRegulatorGain(self, CurrentRegulatorGain):
		_CurrentRegulatorGain = ctypes.c_double(CurrentRegulatorGain)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setCurrentRegulatorGain
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _CurrentRegulatorGain)

		if result > 0:
			raise PhidgetException(result)


	def getMinCurrentRegulatorGain(self):
		_MinCurrentRegulatorGain = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinCurrentRegulatorGain
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinCurrentRegulatorGain))

		if result > 0:
			raise PhidgetException(result)

		return _MinCurrentRegulatorGain.value

	def getMaxCurrentRegulatorGain(self):
		_MaxCurrentRegulatorGain = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxCurrentRegulatorGain
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxCurrentRegulatorGain))

		if result > 0:
			raise PhidgetException(result)

		return _MaxCurrentRegulatorGain.value

	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def enableFailsafe(self, failsafeTime):
		_failsafeTime = ctypes.c_uint32(failsafeTime)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_enableFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _failsafeTime)

		if result > 0:
			raise PhidgetException(result)


	def getMinFailsafeTime(self):
		_MinFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MinFailsafeTime.value

	def getMaxFailsafeTime(self):
		_MaxFailsafeTime = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxFailsafeTime
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxFailsafeTime))

		if result > 0:
			raise PhidgetException(result)

		return _MaxFailsafeTime.value

	def getFanMode(self):
		_FanMode = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getFanMode
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_FanMode))

		if result > 0:
			raise PhidgetException(result)

		return _FanMode.value

	def setFanMode(self, FanMode):
		_FanMode = ctypes.c_int(FanMode)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setFanMode
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _FanMode)

		if result > 0:
			raise PhidgetException(result)


	def resetFailsafe(self):
		__func = PhidgetSupport.getDll().PhidgetDCMotor_resetFailsafe
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def getTargetBrakingStrength(self):
		_TargetBrakingStrength = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getTargetBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TargetBrakingStrength))

		if result > 0:
			raise PhidgetException(result)

		return _TargetBrakingStrength.value

	def setTargetBrakingStrength(self, TargetBrakingStrength):
		_TargetBrakingStrength = ctypes.c_double(TargetBrakingStrength)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setTargetBrakingStrength
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _TargetBrakingStrength)

		if result > 0:
			raise PhidgetException(result)


	def getTargetVelocity(self):
		_TargetVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getTargetVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_TargetVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _TargetVelocity.value

	def setTargetVelocity(self, TargetVelocity):
		_TargetVelocity = ctypes.c_double(TargetVelocity)

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setTargetVelocity
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

		__func = PhidgetSupport.getDll().PhidgetDCMotor_setTargetVelocity_async
		__func(self.handle, _TargetVelocity, _asyncHandler, _ctx)


	def getVelocity(self):
		_Velocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Velocity))

		if result > 0:
			raise PhidgetException(result)

		return _Velocity.value

	def getMinVelocity(self):
		_MinVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMinVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _MinVelocity.value

	def getMaxVelocity(self):
		_MaxVelocity = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetDCMotor_getMaxVelocity
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxVelocity))

		if result > 0:
			raise PhidgetException(result)

		return _MaxVelocity.value
