import numpy as np
import os
import sys
from utils.roblib import *  # Importation des fonctions nécessaires

# Ajouter le chemin vers le dossier des drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv

class IMU:
    def __init__(self, calibration_file="imu_calibration.npz", gyro_calib_file="gyro_calibration.npz"):
        self.imu = imudrv.Imu9IO()
        self.A_mag = None
        self.b_mag = None
        self.A_acc = None
        self.b_acc = None
        self.gyro_offset = np.zeros((3, 1))  # Par défaut, pas de correction s'il n'y a pas de fichier de calibration

        # Chargement des calibrations
        self.load_calibration(calibration_file)
        self.load_gyro_calibration(gyro_calib_file)

        # Initialisation de la verticale
        self.g_est = np.array([[0], [0], [1]])
        self.lambda_obs = 0.  # Facteur d'atténuation de l'observateur

    def load_calibration(self, filename):
        """Charge les matrices de calibration pour le magnétomètre et l'accéléromètre."""
        data = np.load(filename)
        self.A_mag = data["A_mag"]
        self.b_mag = data["b_mag"].reshape(3,1)
        self.A_acc = data["A_acc"]
        self.b_acc = data["b_acc"].reshape(3,1)
        print("Calibration IMU chargée depuis", filename)

    def load_gyro_calibration(self, filename):
        """Charge l'offset du gyroscope depuis un fichier."""
        if os.path.exists(filename):
            data = np.load(filename)
            self.gyro_offset = data["gyro_offset"].reshape(3, 1)
            print("Calibration du gyroscope chargée :", self.gyro_offset.flatten())
        else:
            print("Fichier", filename, "introuvable, le gyroscope ne sera pas corrigé.")

    def get_corrected_measurements(self):
        """Retourne les mesures corrigées du magnétomètre et de l'accéléromètre."""
        mag_raw = np.array(self.imu.read_mag_raw()).reshape(3, 1)
        acc_raw = np.array(self.imu.read_accel_raw()).reshape(3, 1)

        mag_corrected = np.linalg.inv(self.A_mag) @ (mag_raw + self.b_mag)
        acc_corrected = np.linalg.inv(self.A_acc) @ (acc_raw + self.b_acc)

        return mag_corrected, acc_corrected

    def get_corrected_gyro(self):
        """Retourne les mesures du gyroscope corrigées de l'offset."""
        gyro_raw = np.array(self.imu.read_gyro_raw()).reshape(3, 1)
        gyro_corrected = gyro_raw - self.gyro_offset
        return gyro_corrected

    def estimate_vertical(self, gyro, acc):
        """Applique l'observateur de Luenberger pour estimer la verticale."""
        dt = 0.01  # Intervalle de temps
        skew_w = adjoint(gyro.flatten())

        self.g_est = self.lambda_obs * (np.eye(3) - dt * skew_w) @ self.g_est + (1 - self.lambda_obs) * acc
        self.g_est /= np.linalg.norm(self.g_est)  # Normalisation

        return self.g_est

    def get_euler_angles(self):
        """Calcule les angles d'Euler (roll, pitch, yaw) avec correction du gyroscope."""
        mag, acc = self.get_corrected_measurements()
        gyro = self.get_corrected_gyro()  # Utilise le gyroscope corrigé
        #print("Gyro corrigé:", gyro.flatten())

        g1 = self.estimate_vertical(gyro, acc)
        
        roll = np.arcsin(np.dot([0, 1, 0], g1.flatten()))
        pitch = -np.arcsin(np.dot([1, 0, 0], g1.flatten()))
        
        Rh = rotuv(g1, np.array([[0], [0], [1]]))
        mag_horizontal = Rh @ mag
        yaw = -np.arctan2(mag_horizontal[1, 0], mag_horizontal[0, 0])
        
        return roll, pitch, yaw

# Exemple d'utilisation
if __name__ == "__main__":
    imu = IMU()
    roll, pitch, yaw = imu.get_euler_angles()
    print("Yaw:", np.degrees(yaw))
