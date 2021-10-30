import sys
import ctypes
class StopBits:
	# One Stop Bit
	STOP_BITS_ONE = 1
	# Two Stop Bits
	STOP_BITS_TWO = 2

	@classmethod
	def getName(self, val):
		if val == self.STOP_BITS_ONE:
			return "STOP_BITS_ONE"
		if val == self.STOP_BITS_TWO:
			return "STOP_BITS_TWO"
		return "<invalid enumeration value>"
