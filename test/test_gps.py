import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


from calibration import gyro_calib, magnetometer_calib
from utils import ddlib

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv


GYRO_CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '..',"gyro_calibration.npz")
print(GYRO_CALIBRATION_FILE)


gps = ddlib.GPS()
imu = ddlib.IMU()

#while True:
#    time.sleep(0.05)
#    gps.get_coords()
#    print("coordonnées gps :", gps.gps_position,
#          " | coordonnées cartésiennes :", gps.x, gps.y, end="\r")
    

arduino = arddrv.ArduinoIO()

Nav = ddlib.Navigation(imu, gps, arduino)

Nav.follow_gps((48.1989261, -3.0146283), False)


