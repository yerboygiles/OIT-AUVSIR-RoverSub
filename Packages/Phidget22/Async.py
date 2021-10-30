import sys
import ctypes
from Phidget22.PhidgetSupport import PhidgetSupport

class AsyncSupport:
	__callbacks = {}
	__nativecallback = None

	@staticmethod
	def add(entry, phid):
		t = (entry, phid);
		AsyncSupport.__callbacks[id(t)] = t
		return id(t)

	@staticmethod
	def __getAndRemove(id):
		entry = AsyncSupport.__callbacks.get(id)
		del AsyncSupport.__callbacks[id]
		return entry

	@staticmethod
	def __async_callback(handle, ctx, res):

		if ctx == None:
			return

		# look up callback
		entry = AsyncSupport.__getAndRemove(ctx)
		if entry == None:
			return

		_code = ctypes.c_int(res)
		_desc = ctypes.c_char_p()
		result = PhidgetSupport.getDll().Phidget_getErrorDescription(_code, ctypes.byref(_desc))
		details = _desc.value
		entry[0](entry[1], res, details)

	@staticmethod
	def getCallback():
		if AsyncSupport.__nativecallback is None:
			if sys.platform == 'win32':
				AsyncSupport.__nativecallback = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)(AsyncSupport.__async_callback)
			else:
				AsyncSupport.__nativecallback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)(AsyncSupport.__async_callback)
		return AsyncSupport.__nativecallback