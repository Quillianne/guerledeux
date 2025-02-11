import os, sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from calibration import gyro_calib, magnetometer_calib
from utils import ddlib



GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '..',"gyro_calibration.npz")
print(GYRO_CALIBRATION_FILE)





gps = ddlib.GPS()
while True:
    gps.get_coords()
    print("coordonnées gps : ", gps.last_gps_data)
