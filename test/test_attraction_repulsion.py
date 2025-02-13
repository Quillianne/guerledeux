import os, sys
import time
import numpy as np
import socket

from utils import ddlib_consensus as ddlib
from settings import POINT_BOUEE, FREQUENCE_CIRCLE, RAYON_CIRCLE
import utils.geo_conversion as geo_conversion

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


import arduino_driver_v2 as arddrv


gps = ddlib.GPS()
imu = ddlib.IMU()
arduino = arddrv.ArduinoIO()

Nav = ddlib.Navigation(imu, gps, arduino, Kp=2)

Nav.attraction_repulsion("05")
time.sleep(10)
Nav.return_home()