import sys
import ctypes
class ErrorCode:
	# Call succeeded.
	EPHIDGET_OK = 0
	# Not Permitted
	EPHIDGET_PERM = 1
	# The specified entity does not exist. This is usually a result of Net or Log API calls.
	EPHIDGET_NOENT = 2
	# Call has timed out. This can happen for a number of common reasons: Check that the Phidget you are trying to open is plugged in, and that the addressing parameters have been specified correctly. Check that the Phidget is not already open in another program, such as the Phidget Control Panel, or another program you are developing. If your Phidget has a plug or terminal block for external power, ensure it is plugged in and powered. If you are using remote Phidgets, ensure that your computer can access the remote Phidgets using the Phidget Control Panel. If you are using remote Phidgets, ensure you have enabled Server Discovery or added the server corresponding to the Phidget you are trying to open. If you are using Network Server Discovery, try extending the timeout to allow more time for the server to be discovered.
	EPHIDGET_TIMEOUT = 3
	# Keep Alive Failure
	EPHIDGET_KEEPALIVE = 58
	# The operation was interrupted; either from an error, or because the device was closed.
	EPHIDGET_INTERRUPTED = 4
	# IO Issue
	EPHIDGET_IO = 5
	# Memory Issue
	EPHIDGET_NOMEMORY = 6
	# Access to the resource (file) is denied. This can happen when enabling logging.
	EPHIDGET_ACCESS = 7
	# Address Issue
	EPHIDGET_FAULT = 8
	# Specified resource is in use. This error code is not normally used.
	EPHIDGET_BUSY = 9
	# Object Exists
	EPHIDGET_EXIST = 10
	# Object is not a directory
	EPHIDGET_NOTDIR = 11
	# Object is a directory
	EPHIDGET_ISDIR = 12
	# Invalid or malformed command. This can be caused by sending a command to a device which is not supported in it's current configuration.
	EPHIDGET_INVALID = 13
	# Too many open files in system
	EPHIDGET_NFILE = 14
	# Too many open files
	EPHIDGET_MFILE = 15
	# The provided buffer argument size is too small.
	EPHIDGET_NOSPC = 16
	# File too Big
	EPHIDGET_FBIG = 17
	# Read Only Filesystem
	EPHIDGET_ROFS = 18
	# Read Only Object
	EPHIDGET_RO = 19
	# This API call is not supported. For Class APIs this means that this API is not supported by this device. This can also mean the API is not supported on this OS, or OS configuration.
	EPHIDGET_UNSUPPORTED = 20
	# One or more of the parameters passed to the function is not accepted by the channel in its current configuration.
	EPHIDGET_INVALIDARG = 21
	# Try again
	EPHIDGET_AGAIN = 22
	# Not Empty
	EPHIDGET_NOTEMPTY = 26
	# Something unexpected has occured. Enable library logging and have a look at the log, or contact Phidgets support.
	EPHIDGET_UNEXPECTED = 28
	# Duplicated request. Can happen with some Net API calls, such as trying to add the same server twice.
	EPHIDGET_DUPLICATE = 27
	# Bad Credential
	EPHIDGET_BADPASSWORD = 37
	# Network Unavailable
	EPHIDGET_NETUNAVAIL = 45
	# Connection Refused
	EPHIDGET_CONNREF = 35
	# Connection Reset
	EPHIDGET_CONNRESET = 46
	# No route to host
	EPHIDGET_HOSTUNREACH = 48
	# No Such Device
	EPHIDGET_NODEV = 40
	# A Phidget channel object of the wrong channel class was passed into this API call.
	EPHIDGET_WRONGDEVICE = 50
	# Broken Pipe
	EPHIDGET_PIPE = 41
	# Name Resolution Failure
	EPHIDGET_RESOLV = 44
	# The value is unknown. This can happen right after attach, when the value has not yet been recieved from the Phidget. This can also happen if a device has not yet been configured / enabled. Some properties can only be read back after being set.
	EPHIDGET_UNKNOWNVAL = 51
	# This can happen for a number of common reasons. Be sure you are opening the channel before trying to use it. If you are opening the channel, the program may not be waiting for the channel to be attached. If possible use openWaitForAttachment. Otherwise, be sure to check the Attached property of the channel before trying to use it.
	EPHIDGET_NOTATTACHED = 52
	# Invalid or Unexpected Packet
	EPHIDGET_INVALIDPACKET = 53
	# Argument List Too Long
	EPHIDGET_2BIG = 54
	# Bad Version
	EPHIDGET_BADVERSION = 55
	# Channel was closed. This can happen if a channel is closed while openWaitForAttachment is waiting.
	EPHIDGET_CLOSED = 56
	# Device is not configured enough for this API call. Have a look at the must-set properties for this device and make sure to configure them first.
	EPHIDGET_NOTCONFIGURED = 57
	# End of File
	EPHIDGET_EOF = 31
	# Failsafe Triggered on this channel. Close and Re-open the channel to resume operation.
	EPHIDGET_FAILSAFE = 59

	@classmethod
	def getName(self, val):
		if val == self.EPHIDGET_OK:
			return "EPHIDGET_OK"
		if val == self.EPHIDGET_PERM:
			return "EPHIDGET_PERM"
		if val == self.EPHIDGET_NOENT:
			return "EPHIDGET_NOENT"
		if val == self.EPHIDGET_TIMEOUT:
			return "EPHIDGET_TIMEOUT"
		if val == self.EPHIDGET_KEEPALIVE:
			return "EPHIDGET_KEEPALIVE"
		if val == self.EPHIDGET_INTERRUPTED:
			return "EPHIDGET_INTERRUPTED"
		if val == self.EPHIDGET_IO:
			return "EPHIDGET_IO"
		if val == self.EPHIDGET_NOMEMORY:
			return "EPHIDGET_NOMEMORY"
		if val == self.EPHIDGET_ACCESS:
			return "EPHIDGET_ACCESS"
		if val == self.EPHIDGET_FAULT:
			return "EPHIDGET_FAULT"
		if val == self.EPHIDGET_BUSY:
			return "EPHIDGET_BUSY"
		if val == self.EPHIDGET_EXIST:
			return "EPHIDGET_EXIST"
		if val == self.EPHIDGET_NOTDIR:
			return "EPHIDGET_NOTDIR"
		if val == self.EPHIDGET_ISDIR:
			return "EPHIDGET_ISDIR"
		if val == self.EPHIDGET_INVALID:
			return "EPHIDGET_INVALID"
		if val == self.EPHIDGET_NFILE:
			return "EPHIDGET_NFILE"
		if val == self.EPHIDGET_MFILE:
			return "EPHIDGET_MFILE"
		if val == self.EPHIDGET_NOSPC:
			return "EPHIDGET_NOSPC"
		if val == self.EPHIDGET_FBIG:
			return "EPHIDGET_FBIG"
		if val == self.EPHIDGET_ROFS:
			return "EPHIDGET_ROFS"
		if val == self.EPHIDGET_RO:
			return "EPHIDGET_RO"
		if val == self.EPHIDGET_UNSUPPORTED:
			return "EPHIDGET_UNSUPPORTED"
		if val == self.EPHIDGET_INVALIDARG:
			return "EPHIDGET_INVALIDARG"
		if val == self.EPHIDGET_AGAIN:
			return "EPHIDGET_AGAIN"
		if val == self.EPHIDGET_NOTEMPTY:
			return "EPHIDGET_NOTEMPTY"
		if val == self.EPHIDGET_UNEXPECTED:
			return "EPHIDGET_UNEXPECTED"
		if val == self.EPHIDGET_DUPLICATE:
			return "EPHIDGET_DUPLICATE"
		if val == self.EPHIDGET_BADPASSWORD:
			return "EPHIDGET_BADPASSWORD"
		if val == self.EPHIDGET_NETUNAVAIL:
			return "EPHIDGET_NETUNAVAIL"
		if val == self.EPHIDGET_CONNREF:
			return "EPHIDGET_CONNREF"
		if val == self.EPHIDGET_CONNRESET:
			return "EPHIDGET_CONNRESET"
		if val == self.EPHIDGET_HOSTUNREACH:
			return "EPHIDGET_HOSTUNREACH"
		if val == self.EPHIDGET_NODEV:
			return "EPHIDGET_NODEV"
		if val == self.EPHIDGET_WRONGDEVICE:
			return "EPHIDGET_WRONGDEVICE"
		if val == self.EPHIDGET_PIPE:
			return "EPHIDGET_PIPE"
		if val == self.EPHIDGET_RESOLV:
			return "EPHIDGET_RESOLV"
		if val == self.EPHIDGET_UNKNOWNVAL:
			return "EPHIDGET_UNKNOWNVAL"
		if val == self.EPHIDGET_NOTATTACHED:
			return "EPHIDGET_NOTATTACHED"
		if val == self.EPHIDGET_INVALIDPACKET:
			return "EPHIDGET_INVALIDPACKET"
		if val == self.EPHIDGET_2BIG:
			return "EPHIDGET_2BIG"
		if val == self.EPHIDGET_BADVERSION:
			return "EPHIDGET_BADVERSION"
		if val == self.EPHIDGET_CLOSED:
			return "EPHIDGET_CLOSED"
		if val == self.EPHIDGET_NOTCONFIGURED:
			return "EPHIDGET_NOTCONFIGURED"
		if val == self.EPHIDGET_EOF:
			return "EPHIDGET_EOF"
		if val == self.EPHIDGET_FAILSAFE:
			return "EPHIDGET_FAILSAFE"
		return "<invalid enumeration value>"
