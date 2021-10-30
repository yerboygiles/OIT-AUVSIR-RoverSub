import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.SpatialAlgorithm import SpatialAlgorithm
from Phidget22.PhidgetException import PhidgetException

from Phidget22.Phidget import Phidget

class Spatial(Phidget):

	def __init__(self):
		Phidget.__init__(self)
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._AlgorithmDataFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		else:
			self._AlgorithmDataFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		self._AlgorithmData = None
		self._onAlgorithmData = None

		if sys.platform == 'win32':
			self._SpatialDataFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		else:
			self._SpatialDataFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_double)
		self._SpatialData = None
		self._onSpatialData = None

		__func = PhidgetSupport.getDll().PhidgetSpatial_create
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))

		if res > 0:
			raise PhidgetException(res)

	def __del__(self):
		Phidget.__del__(self)

	def _localAlgorithmDataEvent(self, handle, userPtr, quaternion, timestamp):
		if self._AlgorithmData == None:
			return
		quaternion = [quaternion[i] for i in range(4)]
		self._AlgorithmData(self, quaternion, timestamp)

	def setOnAlgorithmDataHandler(self, handler):
		if handler == None:
			self._AlgorithmData = None
			self._onAlgorithmData = None
		else:
			self._AlgorithmData = handler
			self._onAlgorithmData = self._AlgorithmDataFactory(self._localAlgorithmDataEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetSpatial_setOnAlgorithmDataHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onAlgorithmData, None)
		except RuntimeError:
			self._AlgorithmData = None
			self._onAlgorithmData = None

	def _localSpatialDataEvent(self, handle, userPtr, acceleration, angularRate, magneticField, timestamp):
		if self._SpatialData == None:
			return
		acceleration = [acceleration[i] for i in range(3)]
		angularRate = [angularRate[i] for i in range(3)]
		magneticField = [magneticField[i] for i in range(3)]
		self._SpatialData(self, acceleration, angularRate, magneticField, timestamp)

	def setOnSpatialDataHandler(self, handler):
		if handler == None:
			self._SpatialData = None
			self._onSpatialData = None
		else:
			self._SpatialData = handler
			self._onSpatialData = self._SpatialDataFactory(self._localSpatialDataEvent)

		try:
			__func = PhidgetSupport.getDll().PhidgetSpatial_setOnSpatialDataHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onSpatialData, None)
		except RuntimeError:
			self._SpatialData = None
			self._onSpatialData = None

	def getAlgorithm(self):
		_Algorithm = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetSpatial_getAlgorithm
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Algorithm))

		if result > 0:
			raise PhidgetException(result)

		return _Algorithm.value

	def setAlgorithm(self, Algorithm):
		_Algorithm = ctypes.c_int(Algorithm)

		__func = PhidgetSupport.getDll().PhidgetSpatial_setAlgorithm
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Algorithm)

		if result > 0:
			raise PhidgetException(result)


	def getAlgorithmMagnetometerGain(self):
		_AlgorithmMagnetometerGain = ctypes.c_double()

		__func = PhidgetSupport.getDll().PhidgetSpatial_getAlgorithmMagnetometerGain
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_AlgorithmMagnetometerGain))

		if result > 0:
			raise PhidgetException(result)

		return _AlgorithmMagnetometerGain.value

	def setAlgorithmMagnetometerGain(self, AlgorithmMagnetometerGain):
		_AlgorithmMagnetometerGain = ctypes.c_double(AlgorithmMagnetometerGain)

		__func = PhidgetSupport.getDll().PhidgetSpatial_setAlgorithmMagnetometerGain
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _AlgorithmMagnetometerGain)

		if result > 0:
			raise PhidgetException(result)


	def getDataInterval(self):
		_DataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetSpatial_getDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _DataInterval.value

	def setDataInterval(self, DataInterval):
		_DataInterval = ctypes.c_uint32(DataInterval)

		__func = PhidgetSupport.getDll().PhidgetSpatial_setDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DataInterval)

		if result > 0:
			raise PhidgetException(result)


	def getMinDataInterval(self):
		_MinDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetSpatial_getMinDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MinDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MinDataInterval.value

	def getMaxDataInterval(self):
		_MaxDataInterval = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().PhidgetSpatial_getMaxDataInterval
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_MaxDataInterval))

		if result > 0:
			raise PhidgetException(result)

		return _MaxDataInterval.value

	def getHeatingEnabled(self):
		_HeatingEnabled = ctypes.c_int()

		__func = PhidgetSupport.getDll().PhidgetSpatial_getHeatingEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_HeatingEnabled))

		if result > 0:
			raise PhidgetException(result)

		return _HeatingEnabled.value

	def setHeatingEnabled(self, HeatingEnabled):
		_HeatingEnabled = ctypes.c_int(HeatingEnabled)

		__func = PhidgetSupport.getDll().PhidgetSpatial_setHeatingEnabled
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _HeatingEnabled)

		if result > 0:
			raise PhidgetException(result)


	def setMagnetometerCorrectionParameters(self, magneticField, offset0, offset1, offset2, gain0, gain1, gain2, T0, T1, T2, T3, T4, T5):
		_magneticField = ctypes.c_double(magneticField)
		_offset0 = ctypes.c_double(offset0)
		_offset1 = ctypes.c_double(offset1)
		_offset2 = ctypes.c_double(offset2)
		_gain0 = ctypes.c_double(gain0)
		_gain1 = ctypes.c_double(gain1)
		_gain2 = ctypes.c_double(gain2)
		_T0 = ctypes.c_double(T0)
		_T1 = ctypes.c_double(T1)
		_T2 = ctypes.c_double(T2)
		_T3 = ctypes.c_double(T3)
		_T4 = ctypes.c_double(T4)
		_T5 = ctypes.c_double(T5)

		__func = PhidgetSupport.getDll().PhidgetSpatial_setMagnetometerCorrectionParameters
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _magneticField, _offset0, _offset1, _offset2, _gain0, _gain1, _gain2, _T0, _T1, _T2, _T3, _T4, _T5)

		if result > 0:
			raise PhidgetException(result)


	def resetMagnetometerCorrectionParameters(self):
		__func = PhidgetSupport.getDll().PhidgetSpatial_resetMagnetometerCorrectionParameters
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def saveMagnetometerCorrectionParameters(self):
		__func = PhidgetSupport.getDll().PhidgetSpatial_saveMagnetometerCorrectionParameters
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def zeroAlgorithm(self):
		__func = PhidgetSupport.getDll().PhidgetSpatial_zeroAlgorithm
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def zeroGyro(self):
		__func = PhidgetSupport.getDll().PhidgetSpatial_zeroGyro
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)

