import sys
import ctypes
class IOVoltage:
	# Voltage supplied by external device
	IO_VOLTAGE_EXTERN = 1
	# 1.8V
	IO_VOLTAGE_1_8V = 2
	# 2.5V
	IO_VOLTAGE_2_5V = 3
	# 3.3V
	IO_VOLTAGE_3_3V = 4
	# 5.0V
	IO_VOLTAGE_5_0V = 5

	@classmethod
	def getName(self, val):
		if val == self.IO_VOLTAGE_EXTERN:
			return "IO_VOLTAGE_EXTERN"
		if val == self.IO_VOLTAGE_1_8V:
			return "IO_VOLTAGE_1_8V"
		if val == self.IO_VOLTAGE_2_5V:
			return "IO_VOLTAGE_2_5V"
		if val == self.IO_VOLTAGE_3_3V:
			return "IO_VOLTAGE_3_3V"
		if val == self.IO_VOLTAGE_5_0V:
			return "IO_VOLTAGE_5_0V"
		return "<invalid enumeration value>"
