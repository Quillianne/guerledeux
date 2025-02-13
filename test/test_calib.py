import os, sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from calibration.magnetometer_calib import IMUCalibration

def main():
    calib = IMUCalibration()
    calib.calibrate_magnetometer()


if __name__ == "__main__":
    main()
