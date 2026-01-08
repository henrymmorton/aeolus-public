import numpy as np

# Directories
LOG_DIR = "/home/henry/git/aeolus_dev/logs"

# Vehicle Constants
V_LENGTH = 0.4
V_WIDTH = 0.15
WHEEL_RADIUS = 0.05
A2A_LENGTH = 0.2
CAM_HEIGHT = 0.5
CA_OFFSET = 0.1
CAM2IM = 0.1

# State Estimation Constants
CONTROL_COVAR = np.array([[1, 0],
                          [0, 1]])
    
MEASUREMENT_COVAR = np.array(
            [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

STATE_PACKER_FORMAT = "<2q12f"

TEENSY_SAMPLE_PERIOD_THRESHOLD = 0.02

# Camera Constants
# Intrinsic camera matrix (K)
# TODO: Replace these with the real deal
fx = 920
cx = 640
cy = 360
fy = 920

INTRINSIC_CAMERA_MATRIX = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0,  0,  1]])

CAMERA_GRID_Y_OFFSET = 0.6053 # meters
PPM = 1000 # pixels per meter

CAMERA_CALIBRATION_MEASUREMENTS = np.array([[0, 0],
                       [-0.22, 0.22],
                       [0, 0.22],
                       [0.22, 0.22],
                       [0, 0.44],
                       [0.22, 0.44],
                       [-0.22, 0.66],
                       [0, 0.66],
                       [0.22, 0.66]])

CAMERA_CALIBRATION_MEASUREMENTS[:,1] += CAMERA_GRID_Y_OFFSET

CAMERA_CALIBRATION_PIXELS = np.array([[490, 473],
                       [106, 296],
                       [487, 300],
                       [868, 302],
                       [489, 221],
                       [767, 221],
                       [270, 172],
                       [490, 173],
                       [707, 173]])

"""
CAMERA_CALIBRATION_MEASUREMENTS = np.array([[-0.22, 0.22],
                       [0.22, 0.22],
                       [-0.22, 0.66],
                       [0.22, 0.66]])

CAMERA_CALIBRATION_MEASUREMENTS[:,1] += CAMERA_GRID_Y_OFFSET

CAMERA_CALIBRATION_PIXELS = np.array([[106, 296],
                       [868, 302],
                       [270, 172],
                       [707, 173]])
"""

# Teensy Constants
CONTROL_PACKET_HEADER = b'\xAA\x55'
TSTATE_BUFFER_SIZE = 89
TSTATE_BUFFER_FORMAT = '<dffffffdddddffff?'

# Nueral Net Constants
NUM_CLASSES = 6

# Camera Constants
IMAGE_WIDTH = 960
IMAGE_HEIGHT = 544

OUTPUT_IMAGE_WIDTH = 960
OUTPUT_IMAGE_HEIGHT = 544

FRAMERATE = 30 # fps

# Warped Image Width And Height
BEV_FOV_WIDTH = 0.443 * 5
BEV_FOV_HEIGHT = 0.664 * 6

# Lane Detection Constants
MORPH_KERNAL_SIZE = 3
FIT_DEG = 2
MIN_LINE_LENGTH = 100

MAX_PATH_VALIDITY = 4 # meters
PATH_X_RANGE = 3 # meters

# Pure Pursuit Constants
LOOKAHEAD_GAIN = 0.75 # unitless scaling factor
MINIMUM_LOOKAHEAD_DISTANCE = 1 # meters

# Loop Rates
STATE_ESTIMATION_LOOP_PERIOD = 0.005

# Filter Variables
STEERING_FILTER_TAU = 0.1

# Obstacle Avoidance Constants
STOP_TIME_THRESHOLD = 2