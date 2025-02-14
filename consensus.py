import os, sys
import time
import numpy as np
import socket

from utils import ddlib_consensus as ddlib
from settings import POINT_BOUEE, FREQUENCE_CIRCLE, RAYON_CIRCLE
import utils.geo_conversion as geo_conversion

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv


gps = ddlib.GPS()
imu = ddlib.IMU()
arduino = arddrv.ArduinoIO()

Nav = ddlib.Navigation(imu, gps, arduino, Kp=2)

# Nav.follow_gps((48.20010, -3.01573), cartesian = False, distance = 5)
# time.sleep(10)
# Nav.return_home()
hostname = socket.gethostname()
number = int(hostname.split("ddboat")[1])
#Nav.attraction_repulsion_follow(number)
global dephasage
dephasage = (2 * np.pi * 18) / (number-1)
point_devant_ponton = (48.1990856, -3.0155828)

def circle_trajectory(t, R = RAYON_CIRCLE, f = FREQUENCE_CIRCLE, M = point_devant_ponton):  #function that returns the point to reach at time t (cartesian)
    """
    draws a circle of radius R around point M
    """
    a0, a1 = geo_conversion.conversion_spherique_cartesien(M)

    x = R*np.sin(2*np.pi*f*t + dephasage) + a0
    y = R*np.cos(2*np.pi*f*t + dephasage) + a1

    return x,y

def circle_trajectory_dot(t, f = FREQUENCE_CIRCLE, R = RAYON_CIRCLE):  #function that returns the derivative of the point to reach at time t (cartesian)    
    """
    function that returns the derivative of the circle
    """
    x_dot = 2*np.pi*R*f*np.cos(2*np.pi*f*t + dephasage)
    y_dot = -2*np.pi*R*f*np.sin(2*np.pi*f*t + dephasage)

    return x_dot, y_dot




Nav.follow_gps((48.1990856, -3.0155828), cartesian = False, distance = 5)

time.sleep(10)

print("starting trajectory following")
Nav.follow_trajectory(circle_trajectory, circle_trajectory_dot)

Nav.return_home()

#hostname = socket.gethostname()
#number = hostname.split("ddboat")[1]
#Nav.attraction_repulsion_follow(number)