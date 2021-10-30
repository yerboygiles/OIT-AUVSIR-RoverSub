import sys
import ctypes
class SpatialAlgorithm:
	# No AHRS algorithm is used.
	SPATIAL_ALGORITHM_NONE = 0
	# AHRS algorithm, incorporating magnetometer data for yaw correction.
	SPATIAL_ALGORITHM_AHRS = 1
	# IMU algorithm, using gyro and accelerometer, but not magnetometer.
	SPATIAL_ALGORITHM_IMU = 2

	@classmethod
	def getName(self, val):
		if val == self.SPATIAL_ALGORITHM_NONE:
			return "SPATIAL_ALGORITHM_NONE"
		if val == self.SPATIAL_ALGORITHM_AHRS:
			return "SPATIAL_ALGORITHM_AHRS"
		if val == self.SPATIAL_ALGORITHM_IMU:
			return "SPATIAL_ALGORITHM_IMU"
		return "<invalid enumeration value>"
