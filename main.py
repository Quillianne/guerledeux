import time
import os
import numpy as np
from calibration.magnetometer_calib import IMUCalibration
from calibration.gyro_calib import GyroCalibration  # Importation de la calibration du gyroscope
from utils.ddlib import IMU

# Fichiers de calibration
CALIBRATION_FILE = "imu_calibration.npz"
GYRO_CALIBRATION_FILE = "gyro_calibration.npz"

def main():
    # Vérifier si la calibration de l'IMU existe
    if not os.path.exists(CALIBRATION_FILE):
        print("Aucune calibration IMU trouvée, lancement de la calibration...")
        calib = IMUCalibration()
        calib.calibrate_magnetometer()
        calib.calibrate_accelerometer()
        calib.save_calibration(CALIBRATION_FILE)
    else:
        print("Calibration IMU existante trouvée, chargement...")

    # Vérifier si la calibration du gyroscope existe
    if not os.path.exists(GYRO_CALIBRATION_FILE):
        print("Aucune calibration Gyro trouvée, lancement de la calibration...")
        gyro_calib = GyroCalibration()
        gyro_calib.calibrate_gyroscope()
        gyro_calib.save_calibration(GYRO_CALIBRATION_FILE)
    else:
        print("Calibration Gyro existante trouvée, chargement...")

    # Initialisation de l'IMU avec les calibrations
    imu = IMU(calibration_file=CALIBRATION_FILE, gyro_calib_file=GYRO_CALIBRATION_FILE)

    print("Affichage en continu des angles d'Euler (Ctrl+C pour arrêter):")

    try:
        while True:
            euler_measurements = []
            for _ in range(5):
                euler_measurements.append(imu.get_euler_angles())
                time.sleep(0.02)
            roll, pitch, yaw = np.mean(euler_measurements, axis=0)
            print("Yaw:", np.degrees(yaw), "\r", end="")

    except KeyboardInterrupt:
        print("\nArrêt du programme.")

if __name__ == "__main__":
    main()
