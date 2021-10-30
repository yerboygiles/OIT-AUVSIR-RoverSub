import sys
import ctypes
class SPIMode:
	# CPOL = 0 CPHA = 0
	SPI_MODE_0 = 1
	# CPOL = 0 CPHA = 1
	SPI_MODE_1 = 2
	# CPOL = 1 CPHA = 0
	SPI_MODE_2 = 3
	# CPOL = 1 CPHA = 1
	SPI_MODE_3 = 4

	@classmethod
	def getName(self, val):
		if val == self.SPI_MODE_0:
			return "SPI_MODE_0"
		if val == self.SPI_MODE_1:
			return "SPI_MODE_1"
		if val == self.SPI_MODE_2:
			return "SPI_MODE_2"
		if val == self.SPI_MODE_3:
			return "SPI_MODE_3"
		return "<invalid enumeration value>"
