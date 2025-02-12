import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils import ddlib
from settings import DT
from circle_trajectory import circle_trajectory, circle_trajectory_dot

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv

import socket
import sys


# Robot 2 (Client)
def robot2_client_onetime(server_ip):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, 5000))
    print("Connected to DDBoat 17 (Server)")

    # def signal_handler(sig, frame):
    #     print("Closing client connection gracefully...")
    #     client_socket.close()
    #     sys.exit(0)
    
    # signal.signal(signal.SIGINT, signal_handler)

    data = ""
    try:
        while True:
            data = client_socket.recv(1024).decode()
            if not data:
                break
            print("Received GPS Position from DDBoat 17: {}".format(data))
            break
    except Exception as e:
        print("Client error: {}".format(e))
    finally:
        client_socket.close()
    return data



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

def get_master_position():
    return robot2_client_onetime("172.20.25.217")


print("demarrage suivi de trajectoire")
while duration < max_time:
    
    point_serveur = get_master_position()

    navigation.follow_trajectory(lambda t: circle_trajectory(t,M = point_serveur), lambda t: circle_trajectory_dot(t, M = point_serveur), 5)
    duration = time.time()-t0

navigation.gps.export_gpx()
print("fin suivi de trajectoire")