import time
import numpy as np
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.ddlib import IMU, Navigation

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv


# Initialisation des capteurs et des moteurs
imu = IMU(dt = 0.1)
arduino = arddrv.ArduinoIO()

# Création de l'instance de navigation
navigation = Navigation(imu, arduino, Kp=1.5, max_speed=200)

# Définition du cap cible (en degrés)
target_heading = 90  # Exemple : maintenir un cap de 90° (Est)

# Durée du suivi de cap en secondes
duration = 30  

print("Démarrage du suivi de cap...")
navigation.follow_cap(target_heading, duration)
print("Test de suivi de cap terminé.")
