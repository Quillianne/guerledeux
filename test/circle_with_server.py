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
#navigation.trigger_gesture()
max_time = 500
duration = 0
t0 = time.time()
client = ddlib.Client("172.20.25.217", port=5000)


print("demarrage suivi de trajectoire")
while duration < max_time:
    
    point_serveur = client.receive()

    navigation.follow_trajectory(lambda t: circle_trajectory(t,M = point_serveur), lambda t: circle_trajectory_dot(t, M = point_serveur), 5)
    duration = time.time()-t0

navigation.gps.export_gpx()
print("fin suivi de trajectoire")