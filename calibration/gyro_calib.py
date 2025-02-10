import numpy as np
import os, sys
import time

# Ajout du chemin vers le répertoire contenant les drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv

from settings import GYRO_CALIBRATION_FILE

class GyroCalibration:
    def __init__(self):
        self.imu = imudrv.Imu9IO()
        self.gyro_offset = None

    def calibrate_gyroscope(self, num_samples=50):
        """Calibre le gyroscope en prenant la moyenne de plusieurs mesures."""
        print("Acquisition de", num_samples, "mesures du gyroscope...")

        gyro_measurements = []
        for _ in range(num_samples):
            gyro_measurements.append(self.imu.read_gyro_raw())
            time.sleep(0.1)

        self.gyro_offset = np.mean(gyro_measurements, axis=0)

        print("Offset du gyroscope calculé :", self.gyro_offset)

    def save_calibration(self, filename=GYRO_CALIBRATION_FILE):
        """Sauvegarde l'offset du gyroscope."""
        np.savez(filename, gyro_offset=self.gyro_offset)
        print("Calibration du gyroscope sauvegardée dans", filename)

    def load_calibration(self, filename="gyro_calibration.npz"):
        """Charge l'offset du gyroscope."""
        data = np.load(filename)
        self.gyro_offset = data["gyro_offset"]
        print("Calibration du gyroscope chargée depuis", filename)

# Exécution de la calibration
if __name__ == "__main__":
    gyro_calib = GyroCalibration()
    gyro_calib.calibrate_gyroscope()
    gyro_calib.save_calibration()
