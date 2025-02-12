import os, sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from calibration import gyro_calib, magnetometer_calib
from utils import ddlib



client = ddlib.Client("172.20.25.217", port=5000)
client.receive()