# settings.py
import os

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__),"imu_calibration.npz")
GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__),"gyro_calibration.npz")
DT = 0.05
POINT_BOUEE = (48.20010, 3.01573)
FREQUENCE_CIRCLE = 450
RAYON_CIRCLE = 40

