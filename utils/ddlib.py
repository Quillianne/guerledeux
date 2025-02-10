import numpy as np
import os
import sys
import time

from utils.roblib import *  # Importation des fonctions nécessaires

# Ajouter le chemin vers le dossier des drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv

from settings import CALIBRATION_FILE, GYRO_CALIBRATION_FILE

class IMU:
    def __init__(self, calibration_file=CALIBRATION_FILE, gyro_calib_file=GYRO_CALIBRATION_FILE, dt = 0.01):
        self.imu = imudrv.Imu9IO()
        self.A_mag = None
        self.b_mag = None
        self.A_acc = None
        self.b_acc = None
        self.dt = dt
        self.gyro_offset = np.zeros((3, 1))  # Par défaut, pas de correction s'il n'y a pas de fichier de calibration

        # Chargement des calibrations
        self.load_calibration(calibration_file)
        self.load_gyro_calibration(gyro_calib_file)

        # Initialisation de la verticale
        self.g_est = np.array([[0], [0], [1]])
        self.lambda_obs = 0.99  # Facteur d'atténuation de l'observateur

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

        skew_w = adjoint(np.radians(0.1*gyro.flatten()))

        self.g_est = self.lambda_obs * (np.eye(3) - self.dt * skew_w) @ self.g_est + (1 - self.lambda_obs) * acc
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


class Navigation:
    def __init__(self, imu, arduino, Kp=1.0, max_speed=100):
        """
        Initialize Navigation system.
        
        :param imu: IMU instance to get the current heading.
        :param arduino: Arduino instance to control the motors.
        :param Kp: Proportional gain for error correction.
        :param max_speed: Maximum speed for the motors.
        """
        self.imu = imu
        self.arduino = arduino
        self.Kp = Kp  # Proportional gain
        self.max_speed = max_speed  # Maximum motor speed

    def get_current_heading(self):
        """Get the current heading (yaw angle) from the IMU."""
        _, _, yaw = self.imu.get_euler_angles()
        return np.degrees(yaw)  # Convert to degrees

    def follow_cap(self, target_cap, duration):
        """
        Make the boat follow the desired heading for a given duration.

        :param target_cap: Desired heading in degrees.
        :param duration: Time in seconds to maintain the heading.
        """
        start_time = time.time()

        while time.time() - start_time < duration:
            # Get current heading
            current_heading = self.get_current_heading()
            
            # Compute heading error
            error = target_cap - current_heading
            error = (error + 180) % 360 - 180  # Keep error within [-180, 180] degrees
            
            # Compute correction using proportional control
            correction = self.Kp * error

            # Set motor speeds based on correction
            base_speed = self.max_speed * 0.5  # Base speed at 50% max
            left_motor = base_speed - correction
            right_motor = base_speed + correction

            # Clip motor speeds within valid range
            left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
            right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

            # Send speed commands to motors
            self.arduino.send_arduino_cmd_motor(left_motor, right_motor)

            # Debugging output
            print("Current:", round(current_heading, 2), "Target:", target_cap, 
                  "Error:", round(error, 2), "L:", round(left_motor, 2), 
                  "R:", round(right_motor, 2), end="\r")

            time.sleep(self.imu.dt)  # Update rate of 10 Hz

        # Stop the motors after duration
        self.arduino.send_arduino_cmd_motor(0, 0)
        print("Navigation complete. Motors stopped.")



# Exemple d'utilisation
if __name__ == "__main__":
    imu = IMU()
    roll, pitch, yaw = imu.get_euler_angles()
    print("Yaw:", np.degrees(yaw))
