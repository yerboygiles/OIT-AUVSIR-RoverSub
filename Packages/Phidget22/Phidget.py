import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport
from Phidget22.Async import *
from Phidget22.ChannelClass import ChannelClass
from Phidget22.ChannelSubclass import ChannelSubclass
from Phidget22.DeviceClass import DeviceClass
from Phidget22.DeviceID import DeviceID
from Phidget22.ErrorEventCode import ErrorEventCode
from Phidget22.PhidgetException import PhidgetException

class Phidget:

	def __init__(self):
		self.handle = ctypes.c_void_p()

		if sys.platform == 'win32':
			self._AttachFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		else:
			self._AttachFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		self._Attach = None
		self._onAttach = None

		if sys.platform == 'win32':
			self._DetachFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		else:
			self._DetachFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
		self._Detach = None
		self._onDetach = None

		if sys.platform == 'win32':
			self._ErrorFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p)
		else:
			self._ErrorFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p)
		self._Error = None
		self._onError = None

		if sys.platform == 'win32':
			self._PropertyChangeFactory = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p)
		else:
			self._PropertyChangeFactory = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p)
		self._PropertyChange = None
		self._onPropertyChange = None


	def __eq__(self, other):
		return hasattr(other, 'handle') and self.handle.value == other.handle.value

	def __hash__(self):
		return self.handle.value

	def __str__(self):
		_value = (ctypes.c_char * 65536)()
		_valueLen = ctypes.c_int32(65536)
		if self.getIsChannel():
			__func = PhidgetSupport.getDll().channelInfo
		else:
			__func = PhidgetSupport.getDll().deviceInfo
		result = __func(self.handle, ctypes.byref(_value), _valueLen)
		return _value.value.decode('utf- 8')

	def __del__(self):
		__func = PhidgetSupport.getDll().Phidget_delete
		__func.restype = ctypes.c_int32
		res = __func(ctypes.byref(self.handle))
		self.handle = None
		if res > 0:
			raise PhidgetException(res)

	def _localAttachEvent(self, handle, userPtr):
		if self._Attach == None:
			return
		self._Attach(self)

	def setOnAttachHandler(self, handler):
		if handler == None:
			self._Attach = None
			self._onAttach = None
		else:
			self._Attach = handler
			self._onAttach = self._AttachFactory(self._localAttachEvent)

		try:
			__func = PhidgetSupport.getDll().Phidget_setOnAttachHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onAttach, None)
		except RuntimeError:
			self._Attach = None
			self._onAttach = None

	def _localDetachEvent(self, handle, userPtr):
		if self._Detach == None:
			return
		self._Detach(self)

	def setOnDetachHandler(self, handler):
		if handler == None:
			self._Detach = None
			self._onDetach = None
		else:
			self._Detach = handler
			self._onDetach = self._DetachFactory(self._localDetachEvent)

		try:
			__func = PhidgetSupport.getDll().Phidget_setOnDetachHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onDetach, None)
		except RuntimeError:
			self._Detach = None
			self._onDetach = None

	def _localErrorEvent(self, handle, userPtr, Code, Description):
		if self._Error == None:
			return
		Description = Description.decode('utf-8')
		self._Error(self, Code, Description)

	def setOnErrorHandler(self, handler):
		if handler == None:
			self._Error = None
			self._onError = None
		else:
			self._Error = handler
			self._onError = self._ErrorFactory(self._localErrorEvent)

		try:
			__func = PhidgetSupport.getDll().Phidget_setOnErrorHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onError, None)
		except RuntimeError:
			self._Error = None
			self._onError = None

	def _localPropertyChangeEvent(self, handle, userPtr, propertyName):
		if self._PropertyChange == None:
			return
		propertyName = propertyName.decode('utf-8')
		self._PropertyChange(self, propertyName)

	def setOnPropertyChangeHandler(self, handler):
		if handler == None:
			self._PropertyChange = None
			self._onPropertyChange = None
		else:
			self._PropertyChange = handler
			self._onPropertyChange = self._PropertyChangeFactory(self._localPropertyChangeEvent)

		try:
			__func = PhidgetSupport.getDll().Phidget_setOnPropertyChangeHandler
			__func.restype = ctypes.c_int32
			res = __func(self.handle, self._onPropertyChange, None)
		except RuntimeError:
			self._PropertyChange = None
			self._onPropertyChange = None

	@staticmethod
	def finalize(flags):
		_flags = ctypes.c_int32(flags)

		__func = PhidgetSupport.getDll().Phidget_finalize
		__func.restype = ctypes.c_int32
		result = __func(_flags)

		if result > 0:
			raise PhidgetException(result)


	@staticmethod
	def getLibraryVersion():
		_LibraryVersion = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getLibraryVersion
		__func.restype = ctypes.c_int32
		result = __func(ctypes.byref(_LibraryVersion))

		if result > 0:
			raise PhidgetException(result)

		return _LibraryVersion.value.decode('utf-8')

	@staticmethod
	def getLibraryVersionNumber():
		_LibraryVersionNumber = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getLibraryVersionNumber
		__func.restype = ctypes.c_int32
		result = __func(ctypes.byref(_LibraryVersionNumber))

		if result > 0:
			raise PhidgetException(result)

		return _LibraryVersionNumber.value.decode('utf-8')

	@staticmethod
	def resetLibrary():
		__func = PhidgetSupport.getDll().Phidget_resetLibrary
		__func.restype = ctypes.c_int32
		result = __func()

		if result > 0:
			raise PhidgetException(result)


	def getAttached(self):
		_Attached = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getAttached
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Attached))

		if result > 0:
			raise PhidgetException(result)

		return _Attached.value

	def getChannel(self):
		_Channel = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getChannel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Channel))

		if result > 0:
			raise PhidgetException(result)

		return _Channel.value

	def setChannel(self, Channel):
		_Channel = ctypes.c_int(Channel)

		__func = PhidgetSupport.getDll().Phidget_setChannel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _Channel)

		if result > 0:
			raise PhidgetException(result)


	def getChannelClass(self):
		_ChannelClass = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getChannelClass
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ChannelClass))

		if result > 0:
			raise PhidgetException(result)

		return _ChannelClass.value

	def getChannelClassName(self):
		_ChannelClassName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getChannelClassName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ChannelClassName))

		if result > 0:
			raise PhidgetException(result)

		return _ChannelClassName.value.decode('utf-8')

	def getChannelName(self):
		_ChannelName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getChannelName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ChannelName))

		if result > 0:
			raise PhidgetException(result)

		return _ChannelName.value.decode('utf-8')

	def getChannelSubclass(self):
		_ChannelSubclass = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getChannelSubclass
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ChannelSubclass))

		if result > 0:
			raise PhidgetException(result)

		return _ChannelSubclass.value

	def close(self):
		__func = PhidgetSupport.getDll().Phidget_close
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def getDeviceChannelCount(self, cls):
		_cls = ctypes.c_int(cls)
		_count = ctypes.c_uint32()

		__func = PhidgetSupport.getDll().Phidget_getDeviceChannelCount
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _cls, ctypes.byref(_count))

		if result > 0:
			raise PhidgetException(result)

		return _count.value

	def getDeviceClass(self):
		_DeviceClass = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getDeviceClass
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceClass))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceClass.value

	def getDeviceClassName(self):
		_DeviceClassName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getDeviceClassName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceClassName))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceClassName.value.decode('utf-8')

	def getDeviceID(self):
		_DeviceID = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getDeviceID
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceID))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceID.value

	def getDeviceLabel(self):
		_DeviceLabel = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getDeviceLabel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceLabel))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceLabel.value.decode('utf-8')

	def setDeviceLabel(self, DeviceLabel):
		_DeviceLabel = ctypes.create_string_buffer(DeviceLabel.encode('utf-8'))

		__func = PhidgetSupport.getDll().Phidget_setDeviceLabel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceLabel))

		if result > 0:
			raise PhidgetException(result)


	def getDeviceName(self):
		_DeviceName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getDeviceName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceName))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceName.value.decode('utf-8')

	def getDeviceSerialNumber(self):
		_DeviceSerialNumber = ctypes.c_int32()

		__func = PhidgetSupport.getDll().Phidget_getDeviceSerialNumber
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceSerialNumber))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceSerialNumber.value

	def setDeviceSerialNumber(self, DeviceSerialNumber):
		_DeviceSerialNumber = ctypes.c_int32(DeviceSerialNumber)

		__func = PhidgetSupport.getDll().Phidget_setDeviceSerialNumber
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _DeviceSerialNumber)

		if result > 0:
			raise PhidgetException(result)


	def getDeviceSKU(self):
		_DeviceSKU = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getDeviceSKU
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceSKU))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceSKU.value.decode('utf-8')

	def getDeviceVersion(self):
		_DeviceVersion = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getDeviceVersion
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_DeviceVersion))

		if result > 0:
			raise PhidgetException(result)

		return _DeviceVersion.value

	def getHub(self):
		_Hub = ctypes.c_void_p()

		__func = PhidgetSupport.getDll().Phidget_getHub
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Hub))

		if result > 0:
			raise PhidgetException(result)

		__Hub = Phidget()
		__Hub.handle = _Hub
		return __Hub

	def getHubPort(self):
		_HubPort = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getHubPort
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_HubPort))

		if result > 0:
			raise PhidgetException(result)

		return _HubPort.value

	def setHubPort(self, HubPort):
		_HubPort = ctypes.c_int(HubPort)

		__func = PhidgetSupport.getDll().Phidget_setHubPort
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _HubPort)

		if result > 0:
			raise PhidgetException(result)


	def getHubPortCount(self):
		_HubPortCount = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getHubPortCount
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_HubPortCount))

		if result > 0:
			raise PhidgetException(result)

		return _HubPortCount.value

	def getIsChannel(self):
		_IsChannel = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getIsChannel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_IsChannel))

		if result > 0:
			raise PhidgetException(result)

		return _IsChannel.value

	def getIsHubPortDevice(self):
		_IsHubPortDevice = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getIsHubPortDevice
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_IsHubPortDevice))

		if result > 0:
			raise PhidgetException(result)

		return _IsHubPortDevice.value

	def setIsHubPortDevice(self, IsHubPortDevice):
		_IsHubPortDevice = ctypes.c_int(IsHubPortDevice)

		__func = PhidgetSupport.getDll().Phidget_setIsHubPortDevice
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _IsHubPortDevice)

		if result > 0:
			raise PhidgetException(result)


	def getIsLocal(self):
		_IsLocal = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getIsLocal
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_IsLocal))

		if result > 0:
			raise PhidgetException(result)

		return _IsLocal.value

	def setIsLocal(self, IsLocal):
		_IsLocal = ctypes.c_int(IsLocal)

		__func = PhidgetSupport.getDll().Phidget_setIsLocal
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _IsLocal)

		if result > 0:
			raise PhidgetException(result)


	def getIsRemote(self):
		_IsRemote = ctypes.c_int()

		__func = PhidgetSupport.getDll().Phidget_getIsRemote
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_IsRemote))

		if result > 0:
			raise PhidgetException(result)

		return _IsRemote.value

	def setIsRemote(self, IsRemote):
		_IsRemote = ctypes.c_int(IsRemote)

		__func = PhidgetSupport.getDll().Phidget_setIsRemote
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _IsRemote)

		if result > 0:
			raise PhidgetException(result)


	def open(self):
		__func = PhidgetSupport.getDll().Phidget_open
		__func.restype = ctypes.c_int32
		result = __func(self.handle)

		if result > 0:
			raise PhidgetException(result)


	def openWaitForAttachment(self, timeout):
		_timeout = ctypes.c_uint32(timeout)

		__func = PhidgetSupport.getDll().Phidget_openWaitForAttachment
		__func.restype = ctypes.c_int32
		result = __func(self.handle, _timeout)

		if result > 0:
			raise PhidgetException(result)


	def getParent(self):
		_Parent = ctypes.c_void_p()

		__func = PhidgetSupport.getDll().Phidget_getParent
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_Parent))

		if result > 0:
			raise PhidgetException(result)

		__Parent = Phidget()
		__Parent.handle = _Parent
		return __Parent

	def getServerHostname(self):
		_ServerHostname = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getServerHostname
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ServerHostname))

		if result > 0:
			raise PhidgetException(result)

		return _ServerHostname.value.decode('utf-8')

	def getServerName(self):
		_ServerName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getServerName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ServerName))

		if result > 0:
			raise PhidgetException(result)

		return _ServerName.value.decode('utf-8')

	def setServerName(self, ServerName):
		_ServerName = ctypes.create_string_buffer(ServerName.encode('utf-8'))

		__func = PhidgetSupport.getDll().Phidget_setServerName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ServerName))

		if result > 0:
			raise PhidgetException(result)


	def getServerPeerName(self):
		_ServerPeerName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getServerPeerName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ServerPeerName))

		if result > 0:
			raise PhidgetException(result)

		return _ServerPeerName.value.decode('utf-8')

	def getServerUniqueName(self):
		_ServerUniqueName = ctypes.c_char_p()

		__func = PhidgetSupport.getDll().Phidget_getServerUniqueName
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_ServerUniqueName))

		if result > 0:
			raise PhidgetException(result)

		return _ServerUniqueName.value.decode('utf-8')

	def writeDeviceLabel(self, deviceLabel):
		_deviceLabel = ctypes.create_string_buffer(deviceLabel.encode('utf-8'))

		__func = PhidgetSupport.getDll().Phidget_writeDeviceLabel
		__func.restype = ctypes.c_int32
		result = __func(self.handle, ctypes.byref(_deviceLabel))

		if result > 0:
			raise PhidgetException(result)


	ANY_SERIAL_NUMBER = -1

	ANY_HUB_PORT = -1

	ANY_CHANNEL = -1

	ANY_LABEL = None

	INFINITE_TIMEOUT = 0

	DEFAULT_TIMEOUT = 1000
