import os, sys
import time
import numpy as np
import socket


sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils import ddlib_consensus as ddlib
from settings import POINT_BOUEE, FREQUENCE_CIRCLE, RAYON_CIRCLE
import utils.geo_conversion as geo_conversion


sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

import arduino_driver_v2 as arddrv


arduino = arddrv.ArduinoIO()

for i in range(25):
    left_motor = right_motor = i*10
    arduino.send_arduino_cmd_motor(left_motor, right_motor)
    time.sleep(0.07)

arduino.send_arduino_cmd_motor(112, 112)
time.sleep(0.2)
arduino.send_arduino_cmd_motor(0, 0)
time.sleep(0.5)

for i in range(25):
    left_motor = right_motor = i*10
    arduino.send_arduino_cmd_motor(left_motor, right_motor)
    time.sleep(0.07)

arduino.send_arduino_cmd_motor(112, 112)
time.sleep(0.2)
arduino.send_arduino_cmd_motor(0, 0)
time.sleep(0.5)


arduino.send_arduino_cmd_motor(112, 112)
time.sleep(0.2)
arduino.send_arduino_cmd_motor(250, 250)
time.sleep(3)


arduino.send_arduino_cmd_motor(0, 0)


print("Fin de Chantier")