import sys
import ctypes
class Endianness:
	# MSB First
	ENDIANNESS_MSB_FIRST = 1
	# LSB First
	ENDIANNESS_LSB_FIRST = 2

	@classmethod
	def getName(self, val):
		if val == self.ENDIANNESS_MSB_FIRST:
			return "ENDIANNESS_MSB_FIRST"
		if val == self.ENDIANNESS_LSB_FIRST:
			return "ENDIANNESS_LSB_FIRST"
		return "<invalid enumeration value>"
