import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from calibration import gyro_calib, magnetometer_calib
from utils import ddlib



GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '..',"gyro_calibration.npz")
print(GYRO_CALIBRATION_FILE)



gps = ddlib.GPS()
while True:
    time.sleep(0.05)
    gps.get_coords()
    print("coordonnées gps :", gps.gps_position,
          " | coordonnées cartésiennes :", gps.x, gps.y, end="\r")
