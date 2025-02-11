import numpy as np
import os
import sys
import time

from utils.roblib import *  # Importation des fonctions nécessaires
import utils.geo_conversion as geo

# Ajouter le chemin vers le dossier des drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv
import gps_driver_v2 as gpsdrv

from settings import CALIBRATION_FILE, GYRO_CALIBRATION_FILE

class IMU:
    def __init__(self, calibration_file=CALIBRATION_FILE, gyro_calib_file=GYRO_CALIBRATION_FILE, dt = 0.01):
        self.imu_driver = imudrv.Imu9IO()
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
        mag_raw = np.array(self.imu_driver.read_mag_raw()).reshape(3, 1)
        acc_raw = np.array(self.imu_driver.read_accel_raw()).reshape(3, 1)

        mag_corrected = np.linalg.inv(self.A_mag) @ (mag_raw + self.b_mag)
        acc_corrected = np.linalg.inv(self.A_acc) @ (acc_raw + self.b_acc)

        return mag_corrected, acc_corrected

    def get_corrected_gyro(self):
        """Retourne les mesures du gyroscope corrigées de l'offset."""
        gyro_raw = np.array(self.imu_driver.read_gyro_raw()).reshape(3, 1)
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
    def __init__(self, imu, gps, arduino_driver, Kp=1.0, max_speed=100):
        """
        Initialize Navigation system.
        
        :param imu: IMU instance to get the current heading.
        :param gps: GPS instance to get the current coordinates.
        :param arduino: Arduino instance to control the motors.
        :param Kp: Proportional gain for error correction.
        :param max_speed: Maximum speed for the motors.
        """
        self.imu = imu
        self.imu_driver = self.imu.imu_driver
        self.dt = self.imu.dt
        self.gps = gps
        self.arduino_driver = arduino_driver
        self.Kp = Kp  # Proportional gain
        self.max_speed = max_speed  # Maximum motor speed

    def get_z_acc_mean(self, duree = 0.5):
        start_time = time.time()
        mesures = []

        # Capturer les données pendant 'duree' secondes
        while time.time() - start_time < duree:
            mesures.append(self.imu_driver.read_accel_raw())

        # Calculer la moyenne des mesures
        moyenne = np.mean(mesures, axis=0)
        return moyenne[2]

    def trigger_gesture(self):

        while acc_z > 2800:
            self.arduino_driver.send_arduino_cmd_motor(0, 0)
            acc_z = self.get_z_acc_mean(imu)
            #print(acc_z)

        while acc_z < 3500:
            self.arduino_driver.send_arduino_cmd_motor(100, 100)
            acc_z = self.get_z_acc_mean(imu)
            #print(acc_z)

        self.arduino_driver.send_arduino_cmd_motor(0, 0)

    def get_current_heading(self):
        """Get the current heading (yaw angle) from the IMU."""
        _, _, yaw = self.imu.get_euler_angles()
        return np.degrees(yaw)  # Convert to degrees

    def follow_heading(self, target_heading, duration):
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
            error = target_heading - current_heading
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
            self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

            # Debugging output
            print("Target:", target_heading, "Current:", round(current_heading, 2), 
                  "Error:", round(error, 2), end="\r")

            time.sleep(self.dt)  # Update rate of 10 Hz

        # Stop the motors after duration
        self.arduino_driver.send_arduino_cmd_motor(0, 0)
        print("Navigation complete. Motors stopped.")
    
    def follow_trajectory(self, f, fdot, duration = 60):
        """
        Make the boat follow a trajectory defined by a function f(t) and its derivative fdot(t).
        
        :param f: Function that returns the desired position at time t.
        :param fdot: Function that returns the desired velocity at time t.
        """
        t = 0
        t0 = time.time()
        while t < duration:
            t = time.time() - t0
            x, y = f(t) #pos de la cible
            vx, vy = fdot(t) #v de la cible
            px, py = self.gps.get_coords() #pos du bateau
            if px != None and py != None:
                #calcul du cap à viser
                vector_to_target = np.array([x, y]) - np.array([px, py])
                distance = np.linalg.norm(vector_to_target)

                heading_to_follow = np.arctan2(vector_to_target[1], vector_to_target[0])*180/np.pi
                current_heading = self.get_current_heading()

                error = heading_to_follow - current_heading
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                correction = self.Kp * error

                #apliquer une correction proportionelle à la distance de la cible 
                reference_distance = 5
                distance_correction = np.tanh(distance/reference_distance)
                
                #envoyer la vitesse
                base_speed = self.max_speed * 0.5
                left_motor = distance_correction*base_speed + correction
                right_motor = distance_correction*base_speed - correction

                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

                #affichage de l'état
                print("Target:", heading_to_follow, "Current:", round(current_heading, 2), 
                      "Error:", round(error, 2), "Distance:", round(distance, 2), end="\r")
                time.sleep(self.dt)
                
        self.arduino_driver.send_arduino_cmd_motor(0, 0)
        print("Fin de chantier")

    
    def follow_gps(self, target_coords, cartesian=True, distance=5):
        """
        Make the boat follow the desired GPS coordinates for a given duration.

        :param target_coords: Tuple of target GPS coordinates (latitude, longitude).
        :param cartesian: If True, enter cartesian coordinates in the 'target_coords' arg, else use GPS coords.
        :param distance: distance to stop to the target
        """

        # if the target coordinates are in gps
        if not cartesian:
            # convert them to cartesian
            target_coords = geo.conversion_spherique_cartesien(target_coords, lat_m=48.1996872, long_m=-3.0153766, rho=6371000)
        target_coords = np.array(target_coords)

        distance_target = np.inf

        while distance_target > distance:
            
            # get current gps coordinates in cartesian
            current_coords = np.array(self.gps.get_coords())
            
            # Compute heading to target
            delta_coords = target_coords - current_coords
            target_heading = np.degrees(np.atan2(delta_coords))

            # Compute distance to target
            distance_target = np.linalg.norm(delta_coords)

            # get current heading
            current_heading = self.get_current_heading()
            
            # Error
            error = target_heading - current_heading
            error = (error + 180) % 360 - 180  # Keep error within [-180, 180] degrees
            correction = self.Kp * error

            # Proportional command to the motors
            base_speed = self.max_speed * distance_target/10
            left_motor = base_speed - correction
            right_motor = base_speed + correction

            # Clip motor speeds within valid range
            left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
            right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

            # Send speed commands to motors
            self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)
            
            time.sleep(self.imu.dt)  # Update rate of 10 Hz

        # Stop the motors after duration
        self.arduino_driver.send_arduino_cmd_motor(0, 0)

        


class GPS():
    def __init__(self, debug = False):
        self.gps = gpsdrv.GpsIO()
        self.gps.set_filter_speed("0")
        self.gps_position = None
        self.debug = debug
        self.x = None
        self.y = None
        self.gps_history = []

    def get_gps(self):
        """Read GPS data from the serial port."""
        gll_ok, gll_data = self.gps.read_gll_non_blocking()
        if gll_ok:
            if self.debug == True:
                print("Debug gll: ", gll_data)
            latitude = geo.convert_to_decimal_degrees(gll_data[0], gll_data[1])
            longitude = geo.convert_to_decimal_degrees(gll_data[2], gll_data[3])
            if latitude != 0 and longitude != 0:
                self.gps_position = (latitude, longitude)
                self.gps_history.append(self.gps_position)
        return self.gps_position
    
    def get_coords(self):
        """returns the current cartesian coordinates (x,y) of the boat"""
        point = self.get_gps()
        if point != None:
            self.x, self.y = geo.conversion_spherique_cartesien(point)
        return self.x, self.y
    
    def export_gpx(self, filename="output.gpx"):
        """
        Exports the recorded GPS history to a GPX file.

        :param filename: The name/path of the output GPX file.
        """
        # Minimal GPX file header
        gpx_header = """<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="GPS Python Class" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>GPS Track</name>
    <trkseg>
"""
        # Minimal GPX file footer
        gpx_footer = """    </trkseg>
  </trk>
</gpx>
"""

        # Write header
        with open(filename, "w", encoding="utf-8") as f:
            f.write(gpx_header)
            # Write each GPS point
            for lat, lon in self.gps_history:
                f.write('      <trkpt lat="{0}" lon="{1}"></trkpt>\n'.format(lat, lon))
            # Write footer
            f.write(gpx_footer)



# Exemple d'utilisation
if __name__ == "__main__":
    imu = IMU()
    roll, pitch, yaw = imu.get_euler_angles()
    print("Yaw:", np.degrees(yaw))
