# settings.py
import os

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__),"imu_calibration.npz")
GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__),"gyro_calibration.npz")
DT = 0.01

