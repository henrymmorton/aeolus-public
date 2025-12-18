import datetime
import struct
import logging

logger =  logging.getLogger(__name__)

from core.constants import TSTATE_BUFFER_SIZE, TSTATE_BUFFER_FORMAT

class TeensyState:

    def __init__(self):
        # Timestamp code
        self.micros_since_boot = 0
        self.timestamp = None

        # The IMU state variables
        self.x_acc = 0
        self.y_acc = 0
        self.z_acc = 0
        self.yaw_rate = 0
        self.pitch_rate = 0
        self.roll_rate = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        # The ultrasonic distance sensor state variables
        self.ultra_dist = 0
        self.obstc_detected = False

        # The tachometer state variables
        self.rpm = 0

        # Derived state variables
        self.velocity_mag = 0
        self.vel_gx = 0
        self.vel_gy = 0
        self.acc_mag = 0

    def read_state_from_serial(self, serial_port, print_state = False):
        """
        Reads the state from the provided serial port

        param serial_port: The serial port object
        param print_state: Whether to print the result
        """
        # Read the serial data into a packed data object
        data = serial_port.read(TSTATE_BUFFER_SIZE)

        # Unpack the data object
        self.micros_since_boot, self.x_acc, self.y_acc, self.z_acc, self.yaw_rate, self.pitch_rate, self.roll_rate, \
        self.yaw, self.pitch,self.roll, self.rpm, self.ultra_dist, self.velocity_mag, self.vel_gx, self.vel_gy, \
        self.acc_mag, self.obstc_detected = struct.unpack(TSTATE_BUFFER_FORMAT, data)

        # TODO: Determine why rpm isnt being read correctly
        
        if print_state:
            print(f"time since boot: {self.micros_since_boot/1000}s, x_acc: {round(self.x_acc, 2)}, y_acc: {round(self.y_acc, 2)}, z_acc: {round(self.z_acc, 2)}, roll: {round(self.roll, 2)}, pitch: {round(self.pitch, 2)}, yaw: {round(self.yaw, 2)}, obstacle_detected: {self.obstc_detected}, obstacle_distance: {self.ultra_dist}, RPM: {self.rpm}, vMag: {self.velocity_mag}")

    def read_state_from_store(self, state_numpy):
        """
        Reads the teensy state from a recording state store

        param state_numpy: A numpy array of the teensy state
        """
        # Timestamps
        self.timestamp = datetime.datetime.fromtimestamp(state_numpy[0])

        # The IMU state variables
        self.x_acc = state_numpy[1]
        self.y_acc = state_numpy[2]
        self.z_acc = state_numpy[3]
        self.yaw_rate = state_numpy[4]
        self.pitch_rate = state_numpy[5]
        self.roll_rate = state_numpy[6]
        self.yaw = state_numpy[7]
        self.pitch = state_numpy[8]
        self.roll = state_numpy[9]

        # The ultrasonic distance sensor state variables
        self.ultra_dist = state_numpy[10]
        self.obstc_detected = bool(state_numpy[11])

        # The tachometer state variables
        self.rpm = state_numpy[12]

        # Derived state variables
        self.velocity_mag = state_numpy[12]
        self.vel_gx = state_numpy[13]
        self.vel_gy = state_numpy[14]
        self.acc_mag = state_numpy[15]

    def write_state_to_store(self, state_store_path):
        """
        Writes the teensy state to a file

        param state_store_path: The path to the state store file
        """
        with open(state_store_path, "a") as statestore:
            state_string = f"{datetime.datetime.timestamp(self.timestamp)},{self.x_acc},{self.y_acc},{self.z_acc},{self.yaw_rate},{self.pitch_rate},{self.roll_rate},{self.yaw},{self.pitch},{self.roll},{self.ultra_dist},{int(self.obstc_detected)},{self.velocity_mag},{self.vel_gx},{self.vel_gy},{self.acc_mag}"
            statestore.write(state_string)
            statestore.write("\n")

    def __str__(self):

        timestamp_str = self.timestamp.strftime("%M-%S.%f")[:-2]

        accel_string = f"accel: ({self.x_acc}, {self.y_acc}, {self.z_acc}), "
        # angular_velocity_string = f"angular velocity: ({self.yaw_rate}, {self.pitch_rate}, {self.roll_rate}), "
        # rotation_string = f"rotation: ({self.yaw}, {self.pitch}, {self.roll})"

        if self.obstc_detected:
            obstacle_string = f"obstacle detection: obstacle detected at {self.ultra_dist}cm, "
        else:
            obstacle_string = "obstacle detection: no obstacle detected"

        rpm_string = f"rpm: {self.rpm}, "
        velocity_string = f"velocity: (mag: {self.velocity_mag}, globalX: {self.vel_gx}, globalY {self.vel_gy})"
        accel_mag_str = f"accel mag: {self.acc_mag}, "

        state_string = timestamp_str + "\n" + accel_mag_str + accel_string + "\n" + rpm_string + velocity_string + "\n" + obstacle_string
        return state_string

        
