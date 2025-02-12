import numpy as np
import os
import sys
import time
import datetime
import socket
import threading

from utils.roblib import *  # Importation des fonctions nécessaires
import utils.geo_conversion as geo

# Ajouter le chemin vers le dossier des drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
# Ajouter le chemin vers le dossier de log
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'log'))
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
                timestamp = datetime.datetime.now() 
                self.gps_position = (latitude, longitude)
                self.gps_history.append((latitude, longitude, timestamp))
        return self.gps_position
    
    def get_coords(self):
        """returns the current cartesian coordinates (x,y) of the boat"""
        point = self.get_gps()
        if point != None:
            self.x, self.y = geo.conversion_spherique_cartesien(point)
        return self.x, self.y
    
    def export_gpx(self, filename="log/output.gpx"):
        """
        Exports the recorded GPS history to a GPX file, including time stamps.

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

        with open(filename, "w", encoding="utf-8") as f:
            # Write header
            f.write(gpx_header)

            # Write each GPS point: (lat, lon, datetime)
            for lat, lon, dt in self.gps_history:
                # Ensure dt is a string in ISO 8601
                # Example: 2023-02-11T15:04:05Z
                if isinstance(dt, datetime.datetime):
                    # If dt is timezone-aware, it might already be UTC.
                    # Otherwise, you can manually convert or just append "Z".
                    time_str = dt.isoformat() + "Z"
                else:
                    # If you stored dt as a string, ensure it matches ISO 8601
                    time_str = str(dt)

                f.write('      <trkpt lat="{}" lon="{}">\n'.format(lat, lon))
                f.write('        <time>{}</time>\n'.format(time_str))
                f.write( '      </trkpt>\n' )

            # Write footer
            f.write(gpx_footer)

class Navigation:
    def __init__(self, imu, gps, arduino_driver, Kp=1.0, max_speed=250):
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
        self.history = []

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
        acc_z = self.get_z_acc_mean()
        while acc_z < -2800:
            self.arduino_driver.send_arduino_cmd_motor(0, 0)
            acc_z = self.get_z_acc_mean()
            #print(acc_z)

        while acc_z > -3500:
            self.arduino_driver.send_arduino_cmd_motor(100, 100)
            acc_z = self.get_z_acc_mean()
            #print(acc_z)

        self.arduino_driver.send_arduino_cmd_motor(0, 0)

    def get_current_heading(self):
        """Get the current heading (yaw angle) from the IMU in degrees."""
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
    
    def follow_trajectory(self, f, fdot, duration = 500, stop_motor = True):
        """
        Make the boat follow a trajectory defined by a function f(t) and its derivative fdot(t).
        
        :param f: Function that returns the desired position at time t.
        :param fdot: Function that returns the desired velocity at time t.
        """
        time_elapsed = 0
        t0 = time.time()
        while time_elapsed < duration:
            t = time.time()
            time_elapsed = t - t0
            x, y = f(t) #pos de la cible
            vx, vy = fdot(t) #v de la cible
            px, py = self.gps.get_coords() #pos du bateau
            self.history.append((np.array((x,y)),np.array((px,py))))
            if px != None and py != None:
                #calcul du cap à viser
                vector_to_target = np.array([x, y]) - np.array([px, py])
                distance = np.linalg.norm(vector_to_target)

                heading_to_follow = -np.arctan2(vector_to_target[1], vector_to_target[0])*180/np.pi
                current_heading = self.get_current_heading()

                error = current_heading - heading_to_follow
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                correction = self.Kp * error

                #apliquer une correction proportionelle à la distance de la cible 
                reference_distance = 5
                distance_correction = np.tanh(distance/reference_distance)
                #distance_correction = 1
                
                #envoyer la vitesse
                base_speed = self.max_speed * 0.9
                left_motor = distance_correction*base_speed + correction
                right_motor = distance_correction*base_speed - correction

                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

                #affichage de l'état
                print("Vitesse moteur:", round(distance_correction*base_speed,2), "D_Corr:", round(distance_correction, 2), 
                      "Error:", round(error, 2), "Distance:", round(distance, 2), end="\r")
                time.sleep(self.dt)
        if stop_motor == True:        
            self.arduino_driver.send_arduino_cmd_motor(0, 0)
        np.savez("log/trajectory.npz", history=self.history)
        self.history = []
        #print("Fin de chantier")
    
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
            #print(current_coords)
            if current_coords[0] != None and current_coords[1] != None:
                # Compute heading to target
                delta_coords = target_coords - current_coords
                target_heading = -np.degrees(np.arctan2(delta_coords[1], delta_coords[0]))

                # Compute distance to target
                distance_target = np.linalg.norm(delta_coords)


                # get current heading
                current_heading = self.get_current_heading()
                
                # Error
                #print(current_heading)
                #print(target_heading)
                error = current_heading - target_heading
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                correction = self.Kp * error

                reference_distance = 5
                distance_correction = np.tanh(distance_target/reference_distance)

                # Proportional command to the motors
                base_speed = self.max_speed * 0.9

                left_motor = distance_correction*base_speed + correction
                right_motor = distance_correction*base_speed - correction

                # Clip motor speeds within valid range
                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                # Send speed commands to motors
                self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)
                

                print("Vitesse moteur:", round(distance_correction*base_speed,2), "D_Corr:", round(distance_correction, 2), 
                        "Error:", round(error, 2), "Distance:", round(distance_target, 2), end="\r")
                time.sleep(self.dt)

        # Stop the motors after duration
        self.arduino_driver.send_arduino_cmd_motor(0, 0)

    def stay_at(self, point, cartesien=False):
        """
        This function allow the boat to stay at a desired position point
        """
        if not cartesien:
            point = geo.conversion_spherique_cartesien(point)

        current_position = np.array(self.gps.get_coords())
        while True:
            current_position = np.array(self.gps.get_coords())
            if np.linalg.norm(current_position - point) > 5:
                self.follow_gps(point, cartesien=cartesien)
            time.sleep(0.1)

    def follow_boat(self, boat = 18, port = 5000, distance = 5):
        """
        This function allows the boat to follow another boat
        """
        ip = "172.20.25." + str(boat)
        client_boat = Client(ip, port)
        target = None
        current_position = [None, None]

        while current_position[0] == None or current_position[1] == None:
            current_position = np.array(self.gps.get_coords())
            #print(current_position)
            time.sleep(0.1)

        print("position propre bien recuperee") 

        while target == None:
            target = client_boat.receive()
            #print(target)
            time_target_acquired = time.time()
            time.sleep(1)

        print("position cible bien recuperee")

        while True:
            if time.time() - time_target_acquired > 2:
                received = client_boat.receive()
                if received != None:
                    target = received
                    print("-------------------  New target acquired  -------------------", end="\r")
                time_target_acquired = time.time()

            if self.gps.get_coords()[0] != None and self.gps.get_coords()[1] != None and target != None:

                current_position = np.array(self.gps.get_coords())

                #print("position cible :", target, "position propre :", current_position)

                delta_coords = target - current_position
                target_heading = -np.degrees(np.arctan2(delta_coords[1], delta_coords[0]))
                distance_target = np.linalg.norm(delta_coords)

                current_heading = self.get_current_heading()
                
                error = current_heading - target_heading
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                correction = self.Kp * error

                reference_distance = 8
                distance_correction = np.tanh(distance_target/reference_distance)

                # Proportional command to the motors
                base_speed = self.max_speed * 0.9

                if distance_target < 5:
                    distance_correction = 0

                left_motor = distance_correction*base_speed + correction
                right_motor = distance_correction*base_speed - correction

                # Clip motor speeds within valid range
                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)



                # Send speed commands to motors
                self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)
                

                print("Vitesse moteur:", round(distance_correction*base_speed,2), "D_Corr:", round(distance_correction, 2), 
                        "Error:", round(error, 2), "Distance:", round(distance_target, 2), end="\r")
                time.sleep(self.dt)

        # Stop the motors after duration
        #self.arduino_driver.send_arduino_cmd_motor(0, 0)


class Client:
    def __init__(self, server_ip, port=5000):     
        self.host = server_ip
        self.port = port
        self.client = None
        #self.connect()

        self.last_data = None

    def connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.host, self.port))
        #print("connected DDGOAT to server :", self.host)

    def send(self, data):
        pass

    def receive(self):
        self.connect()
        data = self.client.recv(1024)
        #print(data)
        data = data.decode()
        if not data :
            print("server ", self.host, ": no data received")
            return None

        self.last_data = data
        self.client.close()
        #print("received :", data, " from server :", self.host)
        #print("decoded data :", self.serv_decode())
        return self.serv_decode()
    
    def serv_decode(self):
        """returns the last data received from the server to the gps format"""
        decoded_data = self.last_data.split(";")
        decoded_data = (geo.convert_to_decimal_degrees(decoded_data[0], decoded_data[1][0]), geo.convert_to_decimal_degrees(decoded_data[2], decoded_data[3][0]))
        return decoded_data

    def __del__(self):
        if self.client:
            self.client.close()



# Exemple d'utilisation
if __name__ == "__main__":
    imu = IMU()
    roll, pitch, yaw = imu.get_euler_angles()
    print("Yaw:", np.degrees(yaw))
