import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils import ddlib
from settings import DT
from utils import geo_conversion as geo
from circle_trajectory import circle_trajectory, circle_trajectory_dot

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv

import socket
import sys

# Initialisation des capteurs et des moteurs
imu = ddlib.IMU(dt = DT)
arduino = arddrv.ArduinoIO()
gps = ddlib.GPS()

# Création de l'instance de navigation
navigation = ddlib.Navigation(imu, gps, arduino, Kp=1, max_speed=240)

print("demarrage du suivi de file")
navigation.follow_boat(216)
print("fin suivi de bateau")