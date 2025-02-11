import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


from calibration import gyro_calib, magnetometer_calib
from utils import ddlib



GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '..',"gyro_calibration.npz")
print(GYRO_CALIBRATION_FILE)



# gps = ddlib.GPS()
# while True:
#     time.sleep(1)
#     gps.get_coords()
#     print("coordonnées gps :", gps.gps_position,
#           "  ||  coordonnées cartésiennes :", gps.x, gps.y, end="\r")
    


imu = ddlib.IMU()
gps = ddlib.GPS()
arduino = arddrv.ArduinoIO()

Nav = ddlib.Navigation(imu, gps, arduino)

Nav.follow_gps((48.1996872, -3.0153766), False)


