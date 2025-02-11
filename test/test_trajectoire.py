import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils import ddlib
from settings import DT
from circle_trajectory import circle_trajectory, circle_trajectory_dot

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv


# Initialisation des capteurs et des moteurs
imu = ddlib.IMU(dt = DT)
arduino = arddrv.ArduinoIO()
gps = ddlib.GPS()

# Création de l'instance de navigation
navigation = ddlib.Navigation(imu, gps, arduino, Kp=1.5, max_speed=240)
#navigation.trigger_gesture()

print("demarrage suivi de trajectoire")
navigation.follow_trajectory(circle_trajectory, circle_trajectory_dot)
navigation.gps.export_gpx()
print("fin suivi de trajectoire")