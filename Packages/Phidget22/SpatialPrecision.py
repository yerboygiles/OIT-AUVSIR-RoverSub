import sys
import ctypes
class SpatialPrecision:
	# High precision sensor is used when possible, fallback to low precision sensor.
	SPATIAL_PRECISION_HYBRID = 0
	# High precision sensor is always used.
	SPATIAL_PRECISION_HIGH = 1
	# Low precision sensor is always used.
	SPATIAL_PRECISION_LOW = 2

	@classmethod
	def getName(self, val):
		if val == self.SPATIAL_PRECISION_HYBRID:
			return "SPATIAL_PRECISION_HYBRID"
		if val == self.SPATIAL_PRECISION_HIGH:
			return "SPATIAL_PRECISION_HIGH"
		if val == self.SPATIAL_PRECISION_LOW:
			return "SPATIAL_PRECISION_LOW"
		return "<invalid enumeration value>"
