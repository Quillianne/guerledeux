import time
import os
import numpy as np
from calibration.magnetometer_calib import IMUCalibration
from utils.ddlib import IMU

CALIBRATION_FILE = "imu_calibration.npz"

def main():
    # Vérifier si la calibration existe déjà
    if not os.path.exists(CALIBRATION_FILE):
        print("Aucune calibration trouvée, lancement de la calibration...")
        calib = IMUCalibration()
        calib.calibrate_magnetometer()
        calib.calibrate_accelerometer()
        calib.save_calibration(CALIBRATION_FILE)
    else:
        print("Calibration existante trouvée, chargement...")

    # Initialisation de l'IMU avec la calibration
    imu = IMU(calibration_file=CALIBRATION_FILE)

    print("Affichage en continu des angles d'Euler (Ctrl+C pour arrêter):")

    try:
        while True:
            roll, pitch, yaw = imu.get_euler_angles()
            print("Roll:", np.degrees(roll), "Pitch:", np.degrees(pitch), "Yaw:", np.degrees(yaw))
            time.sleep(0.1)  # Mise à jour toutes les 100 ms
    except KeyboardInterrupt:
        print("\nArrêt du programme.")

if __name__ == "__main__":
    main()
