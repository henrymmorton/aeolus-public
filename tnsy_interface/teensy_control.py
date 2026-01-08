import struct
from core.constants import CONTROL_PACKET_HEADER

class TeensyControlSignal:
    """
    The control signal passed from the Jetson to the Teensy (contains a steering angle and speed)
    """
    
    def __init__(self, speed_signal, steering_signal):
        self.speed_signal = speed_signal
        self.steering_signal = steering_signal
    
    def serialize_control_signal(self):
        """
        Constructs a packet for the teensy control signal and serializes it
        """
        # Pack the control signal into a serialized form
        serialized_control_signal = struct.pack('<ff', self.speed_signal, self.steering_signal)

        # Compute a checksum by XORing all the bytes of the signal together
        checksum = 0
        for b in serialized_control_signal:
            checksum ^= b

        return CONTROL_PACKET_HEADER + serialized_control_signal + bytes([checksum])
  

    

