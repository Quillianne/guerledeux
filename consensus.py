import os, sys
import time
import numpy as np

from utils import ddlib_consensus as ddlib

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv


gps = ddlib.GPS()
imu = ddlib.IMU()


try:
    while True:
        roll, pitch, yaw = imu.get_euler_angles()
        print("Yaw:", np.degrees(yaw), "\r", end="")
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nArrêt du programme.")


#arduino = arddrv.ArduinoIO()

#Nav = ddlib.Navigation(imu, gps, arduino)

#Nav.follow_gps((48.19904833333333, -3.0148149999999996), cartesian = False, distance = 6)