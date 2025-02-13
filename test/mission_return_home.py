import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


from calibration import gyro_calib, magnetometer_calib
from utils import ddlib

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv




gps = ddlib.GPS()
imu = ddlib.IMU()

arduino = arddrv.ArduinoIO()

Nav = ddlib.Navigation(imu, gps, arduino)

Nav.return_home()