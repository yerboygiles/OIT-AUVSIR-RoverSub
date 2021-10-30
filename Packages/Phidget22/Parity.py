import sys
import ctypes
class Parity:
	# No Parity Bit
	PARITY_MODE_NONE = 1
	# Even Parity
	PARITY_MODE_EVEN = 2
	# Odd Parity
	PARITY_MODE_ODD = 3

	@classmethod
	def getName(self, val):
		if val == self.PARITY_MODE_NONE:
			return "PARITY_MODE_NONE"
		if val == self.PARITY_MODE_EVEN:
			return "PARITY_MODE_EVEN"
		if val == self.PARITY_MODE_ODD:
			return "PARITY_MODE_ODD"
		return "<invalid enumeration value>"
