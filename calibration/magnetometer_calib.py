import numpy as np
import os, sys

from settings import CALIBRATION_FILE

# Ajout du chemin vers le répertoire contenant les drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv
import time

class IMUCalibration:
    def __init__(self):
        self.imu = imudrv.Imu9IO()
        self.A_mag = None
        self.b_mag = None
        self.A_acc = None
        self.b_acc = None

    def acquire_measurement(self, sensor_type, label):
        """Moyenne sur 10 mesures en attendant l'appui sur Entrée"""
        input("Positionne l'IMU en " + label + " et appuie sur Entrée...")
        measurements = []

        for _ in range(50):
            if sensor_type == "mag":
                measurements.append(self.imu.read_mag_raw())
            elif sensor_type == "acc":
                measurements.append(self.imu.read_accel_raw())
            time.sleep(0.01)

        return np.mean(measurements, axis=0)

    def calibrate_magnetometer(self):
        """Calibre le magnetometre en prenant des mesures dans differentes positions"""
        print("Calibration du magnetometre en cours...")

        # Prendre les mesures
        xN = self.acquire_measurement("mag", "Nord a plat")
        xS = self.acquire_measurement("mag", "Sud a l'envers")
        xW = self.acquire_measurement("mag", "Ouest a plat")
        xU = self.acquire_measurement("mag", "Vertical Z vers le nord")

        # Constantes
        I = np.radians(64)  # Inclinaison du champ magnetique en radians
        beta_mag = 46e-6    # Intensite du champ magnetique en Tesla

        # Calcul du biais (b)
        self.b_mag = -0.5 * (xN + xS)

        # Matrice de reference Y
        Y = beta_mag * np.array([
            [np.cos(I),  0,         -np.sin(I)],
            [0,         -np.cos(I),  0        ],
            [-np.sin(I), -np.sin(I),  np.cos(I)]
        ])

        # Matrice de correction (A)
        X = np.column_stack([xN + self.b_mag, xW + self.b_mag, xU + self.b_mag])
        self.A_mag = np.dot(X, np.linalg.inv(Y))

        print("Calibration du magnetometre terminee.")

    def calibrate_accelerometer(self):
        """Calibre l'accelerometre en prenant des mesures dans differentes positions"""
        print("Calibration de l'accelerometre en cours...")

        # Prendre les mesures
        xx = self.acquire_measurement("acc", "X vers le haut")
        xy = self.acquire_measurement("acc", "Y vers le haut")
        xz = self.acquire_measurement("acc", "Z vers le haut")
        x_negz = self.acquire_measurement("acc", "Z vers le bas")

        # Constante
        beta_acc = 9.8  # Gravite en m/s^2

        # Calcul du biais (b)
        self.b_acc = -0.5 * (xz + x_negz)

        # Matrice de correction (A)
        X = np.column_stack([xx + self.b_acc, xy + self.b_acc, xz + self.b_acc])
        self.A_acc = (1 / beta_acc) * X

        print("Calibration de l'accelerometre terminee.")

    def save_calibration(self, filename=CALIBRATION_FILE):
        """Sauvegarde les matrices de calibration"""
        np.savez(filename, A_mag=self.A_mag, b_mag=self.b_mag, A_acc=self.A_acc, b_acc=self.b_acc)
        print("Calibration sauvegardee dans " + filename)

    def load_calibration(self, filename=CALIBRATION_FILE):
        """Charge les matrices de calibration depuis un fichier"""
        data = np.load(filename)
        self.A_mag = data["A_mag"]
        self.b_mag = data["b_mag"]
        self.A_acc = data["A_acc"]
        self.b_acc = data["b_acc"]
        print("Calibration chargee depuis " + filename)

# Execution de la calibration
if __name__ == "__main__":
    imu_calib = IMUCalibration()
    imu_calib.calibrate_magnetometer()
    imu_calib.calibrate_accelerometer()
    imu_calib.save_calibration()
