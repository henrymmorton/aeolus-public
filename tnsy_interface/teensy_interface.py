import serial
import time
import datetime
import sys
from .teensy_state import TeensyState, TSTATE_BUFFER_SIZE
from .teensy_control import TeensyControlSignal
import logging

logger =  logging.getLogger(__name__)

class TeensyStream:
    """
    The stream between the teensy and the jetson (Used for receiving teensy states and sending control signals)
    """
    
    def __init__(self,
                port='/dev/ttyTHS1',
                baudrate=115200,
                timeout=2):
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.uart1 = None
        self.teensy_boot_time = None

    def get_microseconds(self):
        return time.time_ns() // 1000
    
    def teensy_micros_to_timestamp(self, teensy_micros):
        return self.teensy_boot_time + datetime.timedelta(microseconds=teensy_micros)

    def teensy_handshake(self):
        """
        Establishes the connection to the teensy and gets the boot time of the teensy
        """
        # Record the time you sent the message
        T_jetson_send = self.get_microseconds()
        self.uart1.write(b"SYNC\n")
        logger.info("TEENSY HANDSHAKE: Sent sync message")
        time.sleep(1/100)

        # Read the reply from the teensy
        line = self.uart1.readline()
        line = line.decode('utf-8', errors='replace')
        line = line.strip()

        T_jetson_recieve = self.get_microseconds()

        if line.startswith("SYNC_REPLY"):
            _, teensy_micros_str = line.split()
            T_teensy_recieve_ms = int(teensy_micros_str)

            # Estimate the transmission delay between the teensy and jetson
            transmission_time = T_jetson_recieve - T_jetson_send
            delay = transmission_time / 2

            # Calculate the boot time of the teensy in ms since epoch (in the jetsons frame)
            T_teensy_boot_ms = T_jetson_recieve - delay - T_teensy_recieve_ms

            # Convert the boot time to a datetimeda
            T_teensy_boot = datetime.datetime.fromtimestamp(T_teensy_boot_ms / 1000000)

            self.teensy_boot_time = T_teensy_boot

            return T_teensy_boot
        
        else:
            logger.error("TEENSY HANDSHAKE: Unexpected response from teensy while trying to synchronize times")
            sys.exit(1)


    def spin_up_serial(self):
        """
        Starts up the serial communication between the Jetson and the Teensy
        """
        try:
            self.uart1 = serial.Serial(self.port, self.baudrate, timeout = self.timeout)
            teensy_boot_time = self.teensy_handshake()
            logger.info(f"TEENSY COMMUNICATION: Teensy boot time: {teensy_boot_time}")
            return teensy_boot_time
        except:
            logger.error("TEENSY COMMUNICATION: Failed to start up teensy serial communication")
            sys.exit(1)

    def get_teensy_state(self, print_state = False):
        """
        Reads the latest teensy state signal from the Teensy
        """
        if self.uart1.in_waiting  >= TSTATE_BUFFER_SIZE:
            current_state = TeensyState()
            current_state.read_state_from_serial(serial_port= self.uart1, print_state = print_state)
            current_state.timestamp = self.teensy_micros_to_timestamp(current_state.micros_since_boot)
            return current_state
        else:
            waiting_bytes = self.uart1.in_waiting
            return False
        
    def send_teensy_control_signal(self, control_signal):
        """
        Sends a teensy control signal from the Jetson to the Teensy

        param control_signal (TeensyControlSignal):
        """
        logger.debug(f"TEENSY STREAM: Sending control signal [speed: {control_signal.speed_signal}m/s, steering: {control_signal.steering_signal}]")
        serialized_control_signal = control_signal.serialize_control_signal()
        self.uart1.write(serialized_control_signal)