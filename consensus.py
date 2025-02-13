import os, sys
import time
import numpy as np

from utils import ddlib_consensus as ddlib

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv


gps = ddlib.GPS()
imu = ddlib.IMU()
arduino = arddrv.ArduinoIO()

Nav = ddlib.Navigation(imu, gps, arduino, Kp=2)

#Nav.follow_gps((48.20010, -3.01573), cartesian = False, distance = 5)
Nav.attraction_repulsion()
time.sleep(10)
Nav.return_home()