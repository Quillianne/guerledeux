import numpy as np
import os
import sys
import time
import datetime
import socket
import threading

from utils.roblib import *  # Importing necessary functions
import utils.geo_conversion as geo

# Add path to the drivers folder
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'drivers-ddboat-v2'))
# Add path to the log folder
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'log'))
import imu9_driver_v2 as imudrv
import gps_driver_v2 as gpsdrv



class IMU:
    def __init__(self, dt=0.01):
        self.imu_driver = imudrv.Imu9IO()
        self.A_mag = None
        self.b_mag = None
        self.dt = dt

        # Load calibrations
        self.load_calibration()

        # Initialize vertical direction
        self.g_est = np.array([[0], [0], [1]])

    def load_calibration(self):
        """Load the calibration matrices for the magnetometer and accelerometer."""
        hostname = socket.gethostname()
        number = hostname.split("ddboat")[1]
        filename = "calib_" + number + ".txt"
        with open(filename, "r") as f:
            lines = f.readlines()
        
        lines = [float(line.strip()) for line in lines]
        # Ensure there are at least 12 lines of data
        if len(lines) < 12:
            raise ValueError("The file must contain at least 12 lines of data.")

        # Extract values
        xN = np.array(lines[0:3]).reshape(3, 1)
        xS = np.array(lines[3:6]).reshape(3, 1)
        xW = np.array(lines[6:9]).reshape(3, 1)
        xU = np.array(lines[9:12]).reshape(3, 1)
        
        # Constants
        I = np.radians(64)  # Magnetic field inclination in radians
        beta_mag = 46e-6    # Magnetic field intensity in Tesla

        # Compute bias (b)
        self.b_mag = -0.5 * (xN + xS)

        # Reference matrix Y
        Y = beta_mag * np.array([
            [np.cos(I),  0,         -np.sin(I)],
            [0,         -np.cos(I),  0        ],
            [-np.sin(I), -np.sin(I),  np.cos(I)]
        ])

        # Correction matrix (A)
        X = np.column_stack([xN + self.b_mag, xW + self.b_mag, xU + self.b_mag])
        self.A_mag = np.dot(X, np.linalg.inv(Y))
    
        print("IMU Calibration loaded from", filename)

    def get_corrected_measurements(self):
        """Return the corrected magnetometer and accelerometer measurements."""
        mag_raw = np.array(self.imu_driver.read_mag_raw()).reshape(3, 1)
        acc_raw = np.array(self.imu_driver.read_accel_raw()).reshape(3, 1)

        mag_corrected = np.linalg.inv(self.A_mag) @ (mag_raw + self.b_mag)

        return mag_corrected

    def get_euler_angles(self):
        """Compute the Euler angles (roll, pitch, yaw) with gyroscope correction."""
        mag = self.get_corrected_measurements()  # Use corrected measurements
        yaw = -np.arctan2(mag[1, 0], mag[0, 0])
        
        return 0, 0, yaw


class GPS:
    def __init__(self, debug=False):
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
            if self.debug:
                print("Debug gll:", gll_data)
            latitude = geo.convert_to_decimal_degrees(gll_data[0], gll_data[1])
            longitude = geo.convert_to_decimal_degrees(gll_data[2], gll_data[3])
            if latitude != 0 and longitude != 0:
                timestamp = datetime.datetime.now() 
                self.gps_position = (latitude, longitude)
                self.gps_history.append((latitude, longitude, timestamp))
        return self.gps_position
    
    def get_coords(self):
        """Return the current cartesian coordinates (x, y) of the boat."""
        point = self.get_gps()
        if point is not None:
            self.x, self.y = geo.conversion_spherique_cartesien(point)
        return self.x, self.y
    
    def export_gpx(self, filename="log/output.gpx"):
        """
        Export the recorded GPS history to a GPX file, including timestamps.

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
                # Ensure dt is a string in ISO 8601 format
                if isinstance(dt, datetime.datetime):
                    time_str = dt.isoformat() + "Z"
                else:
                    time_str = str(dt)

                f.write('      <trkpt lat="{}" lon="{}">\n'.format(lat, lon))
                f.write('        <time>{}</time>\n'.format(time_str))
                f.write('      </trkpt>\n')

            # Write footer
            f.write(gpx_footer)


class Navigation:
    def __init__(self, imu, gps, arduino_driver, Kp=1.0, max_speed=250):
        """
        Initialize the Navigation system.
        
        :param imu: IMU instance to get the current heading.
        :param gps: GPS instance to get the current coordinates.
        :param arduino_driver: Arduino instance to control the motors.
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

    def get_z_acc_mean(self, duree=0.5):
        start_time = time.time()
        mesures = []

        # Capture data for 'duree' seconds
        while time.time() - start_time < duree:
            mesures.append(self.imu_driver.read_accel_raw())

        # Compute the average of the measurements
        moyenne = np.mean(mesures, axis=0)
        return moyenne[2]

    def trigger_gesture(self):
        acc_z = self.get_z_acc_mean()
        while acc_z < -2800:
            self.arduino_driver.send_arduino_cmd_motor(0, 0)
            acc_z = self.get_z_acc_mean()
            # print(acc_z)

        while acc_z > -3500:
            self.arduino_driver.send_arduino_cmd_motor(100, 100)
            acc_z = self.get_z_acc_mean()
            # print(acc_z)

        self.arduino_driver.send_arduino_cmd_motor(0, 0)

    def get_current_heading(self):
        """Get the current heading (yaw angle) from the IMU in degrees."""
        _, _, yaw = self.imu.get_euler_angles()
        return np.degrees(yaw)  # Convert to degrees

    def follow_heading(self, target_heading, duration):
        """
        Make the boat follow the desired heading for a given duration.

        :param target_heading: Desired heading in degrees.
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
            base_speed = self.max_speed * 0.5  # Base speed at 50% of max
            left_motor = base_speed - correction
            right_motor = base_speed + correction

            # Clip motor speeds within valid range
            left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
            right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

            # Send speed commands to motors
            self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

            # Debug output
            print("Target:", target_heading, "Current:", round(current_heading, 2),
                  "Error:", round(error, 2), end="\r")

            time.sleep(self.dt)  # Update rate of 10 Hz

        # Stop the motors after duration
        self.arduino_driver.send_arduino_cmd_motor(0, 0)
        print("Navigation complete. Motors stopped.")
    
    def follow_trajectory(self, f, fdot, duration=140, stop_motor=True):
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
            x, y = f(t)  # Target position
            vx, vy = fdot(t)  # Target velocity
            px, py = self.gps.get_coords()  # Boat position
            self.history.append((np.array((x, y)), np.array((px, py))))
            if px is not None and py is not None:
                # Compute heading to target
                vector_to_target = np.array([x, y]) - np.array([px, py])
                distance = np.linalg.norm(vector_to_target)

                heading_to_follow = -np.arctan2(vector_to_target[1], vector_to_target[0]) * 180 / np.pi
                current_heading = self.get_current_heading()

                error = current_heading - heading_to_follow
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                correction = self.Kp * error

                # Apply a correction proportional to the distance to the target 
                reference_distance = 4
                distance_correction = np.tanh(distance / reference_distance)
                # distance_correction = 1
                
                # Send the speed command
                base_speed = self.max_speed * 0.9
                left_motor = distance_correction * base_speed + correction
                right_motor = distance_correction * base_speed - correction

                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

                # Display status
                print("Motor speed:", round(distance_correction * base_speed, 2), "D_Corr:", round(distance_correction, 2),
                      "Error:", round(error, 2), "Distance:", round(distance, 2), end="\r")
                time.sleep(self.dt)
        if stop_motor:
            self.arduino_driver.send_arduino_cmd_motor(0, 0)
        np.savez("trajectory.npz", history=self.history)
        self.history = []
        # print("End of task")
    
    def follow_gps(self, target_coords, cartesian=True, distance=5):
        """
        Make the boat follow the desired GPS coordinates until within a given distance.

        :param target_coords: Tuple of target GPS coordinates (latitude, longitude).
        :param cartesian: If True, input cartesian coordinates in the 'target_coords' argument; otherwise use GPS coordinates.
        :param distance: Distance at which to stop at the target.
        """

        # If the target coordinates are in GPS format
        if not cartesian:
            # Convert them to cartesian
            target_coords = geo.conversion_spherique_cartesien(target_coords)
        target_coords = np.array(target_coords)

        distance_target = np.inf
        while distance_target > distance:
            # Get current GPS coordinates in cartesian
            current_coords = np.array(self.gps.get_coords())
            if current_coords[0] is not None and current_coords[1] is not None:
                # Compute heading to target
                delta_coords = target_coords - current_coords
                target_heading = -np.degrees(np.arctan2(delta_coords[1], delta_coords[0]))

                # Compute distance to target
                distance_target = np.linalg.norm(delta_coords)

                # Get current heading
                current_heading = self.get_current_heading()
                
                # Compute error
                error = current_heading - target_heading
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360
                correction = self.Kp * error

                reference_distance = distance
                distance_correction = np.tanh(distance_target / reference_distance)

                # Proportional command to the motors
                base_speed = self.max_speed * 0.9

                left_motor = distance_correction * base_speed + correction
                right_motor = distance_correction * base_speed - correction

                # Clip motor speeds within valid range
                left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                # Send speed commands to motors
                self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

                print("Motor speed:", round(distance_correction * base_speed, 2), "D_Corr:", round(distance_correction, 2),
                      "Error:", round(error, 2), "Distance:", round(distance_target, 2), end="\r")
                time.sleep(self.dt)

        # Stop the motors after reaching target
        self.arduino_driver.send_arduino_cmd_motor(0, 0)
    
    def return_home(self):
        self.follow_gps((48.1990856, -3.0155828), cartesian=False, distance=6)
        self.follow_gps((48.19904833333333, -3.0148149999999996), cartesian=False, distance=20)

    def stay_at(self, point, cartesien=False):
        """
        Allow the boat to stay at a desired position.
        """
        if not cartesien:
            point = geo.conversion_spherique_cartesien(point)

        current_position = np.array(self.gps.get_coords())
        while True:
            current_position = np.array(self.gps.get_coords())
            if np.linalg.norm(current_position - point) > 5:
                self.follow_gps(point, cartesian=cartesien)
            time.sleep(0.1)

    def follow_boat(self, boat=18, port=5000, distance=5):
        """
        Allow the boat to follow another boat.
        """
        ip = "172.20.25." + str(boat)
        print(ip)
        client_boat = Client(ip, port)
        target = None
        current_position = [None, None]
        self.history = []
        while current_position[0] is None or current_position[1] is None:
            current_position = np.array(self.gps.get_coords())
            time.sleep(0.1)

        print("Own position successfully retrieved")

        while target is None:
            target = client_boat.receive()
            print(target)
            time_target_acquired = time.time()
            time.sleep(1)

        target = np.array(geo.conversion_spherique_cartesien(target))

        print("Target position successfully retrieved")
        try:
            while True:
                if time.time() - time_target_acquired > 2:
                    received = client_boat.receive()
                    if received is not None:
                        target = received
                        target = np.array(geo.conversion_spherique_cartesien(target))
                        print("-------------------  New target acquired  -------------------", end="\r")
                    time_target_acquired = time.time()

                if self.gps.get_coords()[0] is not None and self.gps.get_coords()[1] is not None and target is not None:
                    current_position = np.array(self.gps.get_coords())
                    delta_coords = target - current_position
                    self.history.append((target, current_position))
                    target_heading = -np.degrees(np.arctan2(delta_coords[1], delta_coords[0]))
                    distance_target = np.linalg.norm(delta_coords)

                    current_heading = self.get_current_heading()
                    
                    error = current_heading - target_heading
                    if error > 180:
                        error -= 360
                    elif error < -180:
                        error += 360
                    correction = self.Kp * error

                    reference_distance = 5
                    distance_correction = np.tanh(distance_target / reference_distance)

                    # Proportional command to the motors
                    base_speed = self.max_speed * 0.9

                    if distance_target < 5:
                        distance_correction = 0

                    left_motor = distance_correction * base_speed + correction
                    right_motor = distance_correction * base_speed - correction

                    # Clip motor speeds within valid range
                    left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
                    right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

                    # Send speed commands to motors
                    self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

                    print("Motor speed:", round(distance_correction * base_speed, 2), "D_Corr:", round(distance_correction, 2),
                          "Error:", round(error, 2), "Distance:", round(distance_target, 2), end="\r")
                    time.sleep(self.dt)
        except KeyboardInterrupt:
            self.arduino_driver.send_arduino_cmd_motor(0, 0)
            np.savez("log/follow_boat.npz", history=self.history)
            print("End of navigation")

    def attraction_repulsion(self, num: str, repuls_weight=1.0, attract_weight=1.0, port=5000):
        # Gather the boats used for consensus (stored in config.txt)
        boats = []
        with open("config.txt", "r") as file:
            for line in file:
                if (line == str(num) + "\n") or (line == str(num)):
                    continue
                ip = "172.20.25.2" + line.strip()
                boats.append(Client(ip, int(port)))

        # Initialize variables for attraction and repulsion
        attraction_weight = 1.0
        repulsion_weight = 1.0
        safe_distance = 15.0  # Safe distance to maintain from other boats

        t_last_call = time.time()
        target_heading = 0.0

        while True:
            current_position = np.array(self.gps.get_coords())

            # Verify the data
            if current_position[0] is None or current_position[1] is None:
                current_position = self.gps.gps_position
                if current_position is None:
                    continue

            total_force = np.array([0.0, 0.0])

            # For each boat
            for boat in boats:
                if time.time() - t_last_call < 1.0:
                    continue
                target = boat.receive()
                if target is None or (isinstance(target, tuple) and any(t is None for t in target)):
                    continue

                target_position = np.array(geo.conversion_spherique_cartesien(target))
                delta_position = target_position - current_position
                distance = np.linalg.norm(delta_position)

                # Update the total force
                if distance < safe_distance:
                    total_force -= repulsion_weight * delta_position / distance**2
                else:
                    total_force += attraction_weight * delta_position / distance**2

            # Compute the heading to follow
            target_heading = np.degrees(np.arctan2(total_force[1], total_force[0]))

            # Get current heading
            current_heading = self.get_current_heading()
            
            error = current_heading - target_heading
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            correction = self.Kp * error
            
            # Speed proportional to the magnitude of the total force
            base_speed = self.max_speed * np.linalg.norm(total_force) / 10
            left_motor = base_speed + correction
            right_motor = base_speed - correction

            left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
            right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

            self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

            print("Total Force:", round(total_force[0], 2), round(total_force[1], 2),
                  "Speed:", round(base_speed, 2),
                  "Target heading:", target_heading,
                  "Error:", round(error, 2), end="\r")
            time.sleep(self.dt)
        
    def attraction_repulsion_follow(self, num, port=5000,
                                    repulsion_weight=1.0, attraction_weight=1.0,
                                    follow_weight=1.5, follow_distance=10.0,
                                    safe_distance=15.0):
        """
        Combine the attraction/repulsion among all boats and the "follow" effect
        (boat i follows boat i-1 in a circular order). The boat 'num' is the current boat.

        :param num: Number of the current boat (e.g., 05).
        :param port: TCP port for receiving positions.
        :param repulsion_weight: Weight of the repulsion force when within 'safe_distance'.
        :param attraction_weight: Weight of the attraction force beyond 'safe_distance'.
        :param follow_weight: Weight of the leader-following force.
        :param follow_distance: Desired distance behind the leader.
        :param safe_distance: Distance below which other boats are repelled.
        """
        num = str(num)

        ################################################################
        # 1) Read config.txt to obtain the list "all_boats".
        ################################################################
        all_boats_nums = []
        with open("config.txt", "r") as file:
            for line in file:
                line = line.strip()
                if not line:
                    continue
                all_boats_nums.append(line)

        if len(all_boats_nums) < 2:
            print("ERROR: at least 2 boats are required in config.txt!")
            return

        # Find the index of the current boat in all_boats_nums
        try:
            my_index = all_boats_nums.index(num)
        except ValueError:
            print("ERROR: boat {} is not in config.txt!".format(num))
            return

        # Determine the leader index (boat i follows boat i-1 modulo N)
        leader_index = (my_index - 1) % len(all_boats_nums)
        leader_num = all_boats_nums[leader_index]

        ################################################################
        # 2) Create clients for all boats (except the current boat)
        ################################################################
        boat_clients = {}
        for other_num in all_boats_nums:
            if other_num == num:
                continue
            ip = "172.20.25.2" + str(other_num)
            boat_clients[other_num] = Client(ip, port)

        ################################################################
        # 3) Mechanism to avoid querying the same boat more than once per second
        ################################################################
        last_time_asked = {bn: 0.0 for bn in boat_clients.keys()}
        last_positions = {bn: None for bn in boat_clients.keys()}

        from math import atan2, degrees

        print("[Boat {}] Starting attraction_repulsion_follow with leader={}".format(num, leader_num))

        while True:
            current_time = time.time()

            # 3.1) Retrieve the current boat's position (local GPS)
            current_position = np.array(self.gps.get_coords())
            if (current_position[0] is None) or (current_position[1] is None):
                if self.gps.gps_position is None:
                    print("No valid GPS yet, waiting...")
                    time.sleep(1)
                    continue
                else:
                    current_position = np.array(geo.conversion_spherique_cartesien(self.gps.gps_position))

            # 3.2) Update positions of all other boats (max once per second)
            for bn, client in boat_clients.items():
                if (current_time - last_time_asked[bn]) >= 1.0:
                    data = client.receive()  # Format: (lat, lon)
                    if data is not None:
                        last_positions[bn] = np.array(geo.conversion_spherique_cartesien(data))
                    last_time_asked[bn] = current_time

            # 3.3) Compute the attraction/repulsion force
            total_force = np.array([0.0, 0.0])
            for bn, other_pos in last_positions.items():
                if other_pos is None:
                    continue
                delta_pos = other_pos - current_position
                dist = np.linalg.norm(delta_pos)
                if dist < 1e-6:
                    continue

                if dist < safe_distance:
                    total_force -= repulsion_weight * (delta_pos**2) / dist
                else:
                    total_force += attraction_weight * (delta_pos**2) / dist

            # 3.4) Compute the "follow" force (follow the leader)
            leader_pos = last_positions.get(leader_num, None)
            if leader_pos is not None:
                delta_leader = leader_pos - current_position
                dist_leader = np.linalg.norm(delta_leader)
                if dist_leader >= 1e-6:
                    direction_leader = delta_leader / dist_leader
                    if dist_leader > follow_distance:
                        attPull = follow_weight * (dist_leader - follow_distance)
                        total_force += attPull * direction_leader
                    elif dist_leader < follow_distance:
                        repPush = follow_weight * (follow_distance - dist_leader)
                        total_force -= repPush * direction_leader

            # 3.5) Convert the force into a target heading and magnitude
            fx, fy = total_force
            normF = np.linalg.norm(total_force)
            if normF < 1e-6:
                self.arduino_driver.send_arduino_cmd_motor(0, 0)
                time.sleep(self.dt)
                continue

            target_heading = -degrees(atan2(fy, fx))
            current_heading = self.get_current_heading()

            error = current_heading - target_heading
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            correction = self.Kp * error
            base_speed = self.max_speed * (normF / 100.0)
            base_speed = np.clip(base_speed, 0, self.max_speed)

            left_motor = base_speed + correction
            right_motor = base_speed - correction

            left_motor = np.clip(left_motor, -self.max_speed, self.max_speed)
            right_motor = np.clip(right_motor, -self.max_speed, self.max_speed)

            self.arduino_driver.send_arduino_cmd_motor(left_motor, right_motor)

            print("[Boat {}] NormF={} -> spd={} TH={} Err={}".format(
                num, round(normF, 2), round(base_speed, 2), round(target_heading, 2), round(error, 2)), end="\r")

            time.sleep(self.dt)



class Client:
    def __init__(self, server_ip, port=5000):
        self.host = server_ip
        self.port = port
        self.client = None
        self.last_data = None

    def connect(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.host, self.port))
        # print("Connected to server:", self.host)

    def send(self, data):
        pass

    def receive(self):
        """Return the last data received from the server, as a GPS pair."""
        self.connect()
        data = self.client.recv(1024)
        data = data.decode()
        if not data:
            print("Server", self.host, ": no data received")
            return None

        self.last_data = data
        self.client.close()
        return self.serv_decode()
    
    def serv_decode(self):
        """Return the last data received from the server in GPS format."""
        decoded_data = self.last_data.split(";")
        decoded_data = (geo.convert_to_decimal_degrees(decoded_data[0], decoded_data[1][0]),
                        geo.convert_to_decimal_degrees(decoded_data[2], decoded_data[3][0]))
        return decoded_data

    def __del__(self):
        if self.client:
            self.client.close()



# Example usage
if __name__ == "__main__":
    imu = IMU()
    roll, pitch, yaw = imu.get_euler_angles()
    print("Yaw:", np.degrees(yaw))
