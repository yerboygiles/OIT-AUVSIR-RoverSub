import sys
import ctypes
class HandshakeMode:
	# RTS is never set, and CTS is ignored.
	HANDSHAKE_MODE_NONE = 1
	# RTS Pin requests to transmit data. CTS input confirms we can send data.
	HANDSHAKE_MODE_REQUEST_TO_SEND = 2
	# RTS signals this device can receive data. CTS confirms we can send data.
	HANDSHAKE_MODE_READY_TO_RECEIVE = 3

	@classmethod
	def getName(self, val):
		if val == self.HANDSHAKE_MODE_NONE:
			return "HANDSHAKE_MODE_NONE"
		if val == self.HANDSHAKE_MODE_REQUEST_TO_SEND:
			return "HANDSHAKE_MODE_REQUEST_TO_SEND"
		if val == self.HANDSHAKE_MODE_READY_TO_RECEIVE:
			return "HANDSHAKE_MODE_READY_TO_RECEIVE"
		return "<invalid enumeration value>"
