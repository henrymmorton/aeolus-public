import os
import argparse
import psutil
import torch.multiprocessing as mp
import torch
import threading
from datetime import datetime
import time
import cv2
import numpy as np

from .vhcl.vehicle import Vehicle
from .cam.camera import PiCameraStream
from .fkr import camera_faker
from .fkr import teensy_faker
from .tnsy_interface.teensy_control import TeensyControlSignal
from .tnsy_interface.teensy_interface import TeensyStream
from .cv.nueral_net.vision_pipeline.ERFNetPredictor import ERFNetPredictor
from .cv.nueral_net.vision_pipeline.lane_detection import Lane, lane_detection_pipeline
from .cv.nueral_net.vision_pipeline.perspective_transform_measured import CameraPerspectiveTransform
from .sc.purepursuit import PurePursuit, pure_pursuit_pipeline
from .se.stateestimation import VehicleState
from .sc.speed_parser import SpeedArray
from. autonomy_processes import neural_net_lane_detection_process, state_estimation_control_process

from .constants import V_LENGTH, V_WIDTH, WHEEL_RADIUS, A2A_LENGTH, CAM_HEIGHT, CA_OFFSET, CAM2IM, CONTROL_COVAR, MEASUREMENT_COVAR, INTRINSIC_CAMERA_MATRIX, CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, PPM, BEV_FOV_WIDTH, BEV_FOV_HEIGHT, CAMERA_GRID_Y_OFFSET

import logging
from .logging_config import LOGGING_CONFIG

# Setup logging
logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("core")
handlers = logger.handlers[:]
logger.handlers.clear()

model_path = "/home/henry/git/aeolus_dev/test/test_models/erfnet_small_11_2.ts"

# hard_image_path = "/home/henry/git/aeolus_dev/test/recording_07_29_small/images/frame_345.png"
hard_image_path = "/home/henry/git/aeolus_dev/test/recording2025-11-01_17-21-34/images/frame_27.png"

hard_recording_path = "/home/henry/git/aeolus_dev/test/recording2025-11-01_17-21-34"

RECORDING_FOLDER = "/home/henry/git/aeolus_dev/test"

HARD_SPEED_MATRIX = (0,[(30, 1), (30, 2), (30, 1.5), (30, 0.5)])

class DebugQueueListener(logging.handlers.QueueListener):
    def handle(self, record):
        print("QueueListener recieved record:", record.name, record.levelname, record.msg)
        super().handle(record)

#Description: The high level program which unites computer functions
class Autonomy:
    """
    The digital model of vehicle autnomy
    """

    def __init__(self, sm, vehicle):
        """
        Default constructor

        param sm: The speed matrix containing speeds and their associated distances or times
        """
        logger.info("AUTONOMY CLASS: Constructing autonomy")
        #----------Timing Data-----------#
        self.startup_time = datetime.now()
        self.teensy_boot_time = None
        self.time = 0
        
        #---------Hardware Data--------#
        self.camera = PiCameraStream()
        self.teensy_streamer = TeensyStream()
        
        #----------Planning Data----------#
        #The speed matrix containing speeds and their associated distances or times
        self.sm = SpeedArray(sm[0], sm[1])

        #----------Vehicle----------#
        self.vehicle = vehicle

        #----------Subsystem Objects----------#
        device = torch.device('cpu')
        self.model_path = model_path
        self.perspective_transformer = CameraPerspectiveTransform(CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, PPM, BEV_FOV_WIDTH, BEV_FOV_HEIGHT, CAMERA_GRID_Y_OFFSET)

        # ----Multiprocessing/Multithreading Values----#
        self.speed_signal = mp.Value('f', 0)
        self.str_angle = mp.Value('f', 0)

        #----Multiprocessing/Multithreading Queues----#
        self.teensy_state_queue = mp.Queue(maxsize=500)
        self.image_queue = mp.Queue(maxsize=2)
        self.lane_queue = mp.Queue(maxsize=2)
        self.state_queue = mp.Queue(maxsize=2)

        logger.info("AUTONOMY CLASS: Finished constructing autonomy")

    #------------ True Input Processes ------------#
    def teensy_communication_process(self, timeout_flag):
        """
        Read the state information from the teensy via serial

        param timeout_flag: A multiprocessing event signaling when to halt the thread
        """
        tsr_loop_count = 0
        while not timeout_flag.is_set():

            # logger.debug(f"TEENSY_STATE_READER_THREAD_STATUS: Teensy state reader loop {tsr_loop_count}")
            current_teensy_state = self.teensy_streamer.get_teensy_state(print_state=False)
            if current_teensy_state:
                self.teensy_state_queue.put(current_teensy_state)

            control_speed = self.speed_signal.value
            steering_angle = self.str_angle.value
        
            # Create the control signal object
            control_signal = TeensyControlSignal(control_speed, steering_angle)
            
            # Send the control signal to the Teensy
            self.teensy_streamer.send_teensy_control_signal(control_signal)
            
    def camera_capture_process(self, timeout_flag):
        """
        Capture images using the Raspberry Pi Camera

        param timeout_flag: A threading event signaling when to halt the thread
        """
        cc_loop_count = 0
        while not timeout_flag.is_set():
            logger.debug(f"CAMERA_CAPTURE_THREAD STATUS: Camera capture loop {cc_loop_count}")
            current_frame, capture_time = self.camera.get_frame()
            frame_tuple = (current_frame, capture_time)
            self.image_queue.put(frame_tuple)
            cc_loop_count += 1

    #------------ Recording Processes ------------#
    def teensy_state_recording_process(self, timeout_flag, state_recording_path):
        """
        Read the state information from the teensy via serial

        param timeout_flag: A threading event signaling when to halt the thread
        param state_recording_path: The path where the state recording should be stored
        """
        while not timeout_flag.is_set():
            current_teensy_state = self.teensy_streamer.get_teensy_state()
            if current_teensy_state:
                current_teensy_state.write_state_to_store(state_recording_path)
            
    def camera_recording_process(self, timeout_flag, image_recording_path):
        """
        Capture images using the Raspberry Pi Camera

        param timeout_flag: A threading event signaling when to halt the thread
        param image_recording_path: The path where the image recording should be stored
        """
        while not timeout_flag.is_set():
            current_frame, capture_time = self.camera.get_frame()
            frame_tuple = (current_frame, capture_time)
            self.camera.mrecord_frame(frame_tuple, image_recording_path)

    #------------ Spoofed Input Processes ------------#
    def camera_spoofing_process(self, start_flag, timeout_flag, camera_stream):
        """
        Streams images from a camera stream

        param start_flag: A multiprocessing flag signaling that the SE/C process has started
        param timeout_flag: A multiprocessing event signaling when to halt the thread
        param camera_stream: A StreamFaker type object for spoofing a camera
        """
        # Wait untill state estimation is up to start
        start_flag.wait()

        while not timeout_flag.is_set():
            current_frame, capture_time = camera_stream.get_video_frame()
            frame_tuple = (current_frame, capture_time)

            # If the image queue is full, drop the oldest image
            if self.image_queue.full():
                self.image_queue.get()
            
            self.image_queue.put(frame_tuple)
            logger.info("Putting new image in queue")

    def teensy_state_spoofing_process(self, start_flag, timeout_flag, teensy_state_store):
        """
        Read the state information from the spoofed teensy state store

        param start_flag: A multiprocessing flag signaling that the SE/C process has started
        param timeout_flag: A multiprocessing event signaling when to halt the thread
        param teensy_state_store: The path to a recorded teensy state file
        """
        # Wait untill state estimation is up to start
        start_flag.wait()

        while not timeout_flag.is_set():
            current_teensy_state = teensy_state_store.get_state_from_state_store()
            if current_teensy_state:
                self.teensy_state_queue.put(current_teensy_state)

            control_speed = self.speed_signal.value
            steering_angle = self.str_angle.value
        
            # Create the control signal object
            control_signal = TeensyControlSignal(control_speed, steering_angle)
            
            # Send the control signal to the Teensy
            # self.teensy_streamer.send_teensy_control_signal(control_signal)
    #------------------------------------------------#


def prime_autonomy_pipeline(aeolus, image_stream = None):
    """
    Run the camera capture -> nueral_net -> state_estimation_process once to establish an initial state

    param aeolus (Autonomy): The declared (but mostly unintializd aeolus object)
    param image_stream: An optional StreamFaker object used for spoofing
    """
    logger.info("AUTONOMY PIPELINE PRIMER: Starting priming pipeline")

    # Grab a frame from the camera
    if image_stream is None:
        current_frame, capture_time = aeolus.camera.get_frame()
    else:
        current_frame, capture_time = image_stream.get_video_frame()

    current_frame = cv2.imread(hard_image_path)

    # Spin up and call the neural net predictor
    predictor = ERFNetPredictor(aeolus.model_path)
    current_inference = predictor.predict_lane_lines(current_frame)

    # Call the lane detection pipeline
    lane_object = lane_detection_pipeline(aeolus.perspective_transformer, current_inference, capture_time, image=current_frame)

    # Declare the initial state
    initial_state = VehicleState(aeolus.vehicle, CONTROL_COVAR, MEASUREMENT_COVAR)

    # Initialize the vehicle state
    initial_state.initialize_state(lane_object)

    # Put the initial state in the queue
    aeolus.state_queue.put(initial_state)

    # Run pure pursuit on the vehicle state to get the initial steering angle
    initial_pursuit = PurePursuit()
    initial_steering_angle = pure_pursuit_pipeline(initial_state, initial_pursuit)

    # Set the initial steering angle
    aeolus.str_angle.value = initial_steering_angle

    logger.info("AUTONOMY PIPELINE PRIMER: Finished priming pipeline")

def spin_up_autonomy(speed_matrix, record = False):
    """
    Initializes all the required objects for an autonomy run and detects the initial state

    param speed_matrix: The matrix of times and speeds which controls vehicle speed in the run
    """
    # Establish the vehicle constants
    my_vehicle = Vehicle(V_LENGTH, V_WIDTH, WHEEL_RADIUS, A2A_LENGTH, CAM_HEIGHT, CA_OFFSET, CAM2IM, INTRINSIC_CAMERA_MATRIX)

    # Initialize the main autonomy object
    my_aeolus_run = Autonomy(speed_matrix, my_vehicle) 

    # Spin up the camera
    my_aeolus_run.camera.spin_up_camera()

    # Spin up the teensy state stream
    teensy_boot_time = my_aeolus_run.teensy_streamer.spin_up_serial()
    my_aeolus_run.teensy_boot_time = teensy_boot_time

    # Prime the pipeline by grabbing an intial image to establish state
    if not record:
        prime_autonomy_pipeline(my_aeolus_run)

    return my_aeolus_run

def spin_up_spoofed_autonomy(speed_matrix, recording_path, sim_now):
    """
    Initializes all the required objects for a replayed autonomy run and detects the initial state

    param speed_matrix: The matrix of times and speeds which controls vehicle speed in the run
    param recording_path: The path to the folder containing the replay data
    param sim_now: Whether to attach historical or current timestamps to spoofed data
    """
    # Establish the vehicle constants
    my_vehicle = Vehicle(V_LENGTH, V_WIDTH, WHEEL_RADIUS, A2A_LENGTH, CAM_HEIGHT, CA_OFFSET, CAM2IM, INTRINSIC_CAMERA_MATRIX)

    # Initialize the main autonomy object
    my_aeolus_run = Autonomy(speed_matrix, my_vehicle)

    # Fetch all the relelvant file paths
    boot_time_path = os.path.join(recording_path, "teensy_boot_time.txt")
    teensy_state_path = os.path.join(recording_path, "teensy_state.txt")
    image_path = os.path.join(recording_path, "images")

    if sim_now:
        spoofed_teensy_boot_time = datetime.now()
    else:
        # Read the teensy boot time
        with open(boot_time_path) as bt_file:
            spoofed_teensy_boot_time = float(bt_file.readline())
            spoofed_teensy_boot_time_datetime = datetime.fromtimestamp(spoofed_teensy_boot_time)
            my_aeolus_run.teensy_boot_time = spoofed_teensy_boot_time_datetime

    # Spin up the spoofed video stream
    image_stream = camera_faker.StreamFaker(image_path, sim_now)

    if sim_now:
        image_stream.start_restamping()

    # Spin up the spoofed teensy state store
    state_store = teensy_faker.TeensyStateStore(teensy_state_path, sim_now)

    # Spin up serial
    # teensy_boot_time = my_aeolus_run.teensy_streamer.spin_up_serial()

    prime_autonomy_pipeline(my_aeolus_run, image_stream)
    
    return my_aeolus_run, image_stream, state_store

def record(speed_matrix=HARD_SPEED_MATRIX):
    """
    The main function for running in recording mode

    param speed_matrix: The matrix of times and speeds which dictates vehicle speed in the run
    """
    # Run startup processes
    aeolus = spin_up_autonomy(speed_matrix, record = True)
    teensy_boot_time = aeolus.teensy_boot_time

    # Set up the directory structure for the recording
    recording_id = f'recording{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'
    recording_path = os.path.join(RECORDING_FOLDER, recording_id)
    boot_time_path = os.path.join(recording_path, "teensy_boot_time.txt")
    teensy_state_path = os.path.join(recording_path, "teensy_state.txt")
    image_path = os.path.join(recording_path, "images")
    os.mkdir(recording_path)
    os.mkdir(image_path)

    with open(boot_time_path, 'a') as bt_file:
        bt_file.write(f"{datetime.timestamp(teensy_boot_time)}")

    # Initialize the different stop flags
    timeout_flag_threading = threading.Event()

    # Initialize teensy state reading thread
    teensy_state_record_thread = threading.Thread(target=aeolus.teensy_state_recording_process, args=[timeout_flag_threading, teensy_state_path])

    # Initialize camera capture thread
    camera_record_thread = threading.Thread(target=aeolus.camera_recording_process, args=[timeout_flag_threading, image_path])

    logger.info("AUTONOMY MAIN (RECORD): Starting threads")
    # Start the measurement threads
    teensy_state_record_thread.start()
    camera_record_thread.start()
    logger.info("AUTONOMY MAIN (RECORD): Finished starting threads")

    # Sleep for the desired recording time
    time.sleep(240)
    logger.info("AUTONOMY MAIN (RECORD): Reached recording timeout, shutting down threads")
    timeout_flag_threading.set()
    teensy_state_record_thread.join()
    camera_record_thread.join()

def autonomy(speed_matrix, spoofing = False, recording_path = None):
    """
    The main function for an autonomy run
    param speed_matrix: The matrix of times and speeds which controls vehicle speed in the run
    param spoofing (bool): Whether or not this is a spoofed run
    param recording_path: Where the replay recording lives for a spoofed run
    """
    # Allocate 4 CPU cores to this process
    p = psutil.Process(os.getpid())
    p.cpu_affinity([0])

    # Create a queue for logs and add a queue handler for the logger
    log_queue = mp.Queue()
    logger.addHandler(logging.handlers.QueueHandler(log_queue))

    # Attach logging to queue listener and start listening
    listener = logging.handlers.QueueListener(log_queue, *handlers)
    listener.start()

    sim_now = True

    # Run startup processes
    if not spoofing:
        aeolus = spin_up_autonomy(speed_matrix)
    else:
        aeolus, video_stream, teensy_state_store = spin_up_spoofed_autonomy(speed_matrix, recording_path, sim_now)

    # Initialize a start flag
    start_flag_mp = mp.Event()

    # Initialize the stop flag
    timeout_flag_mp = mp.Event()

    logger.info("AUTONOMY MAIN: Initializing threads")
    if not spoofing:
        # Initialize teensy state reading thread
        teensy_state_reader_thread = threading.Thread(target=aeolus.teensy_communication_process, args=[timeout_flag_mp,])

        # Initialize camera capture thread
        camera_capture_thread = threading.Thread(target=aeolus.camera_capture_process, args=[timeout_flag_mp,])
    else:
        # Initialize teensy state store spoofing thread
        teensy_state_reader_thread = threading.Thread(target=aeolus.teensy_state_spoofing_process, args=[start_flag_mp, timeout_flag_mp, teensy_state_store,])

        # Initialize camera spoofing thread
        camera_capture_thread = threading.Thread(target=aeolus.camera_spoofing_process, args=[start_flag_mp, timeout_flag_mp, video_stream,])


    logger.info("AUTONOMY MAIN: Finished initializing threads. Initializing processes")
    # TODO: PUt back in real timeout flah as argument once I figure out the FileNotFound problem (see chatGPT)
    # Initialize neural net lane detection process
    neural_net_lane_detection_process_instance = mp.Process(target=neural_net_lane_detection_process, args=(timeout_flag_mp, aeolus.image_queue, aeolus.lane_queue, aeolus.model_path, aeolus.perspective_transformer, log_queue))

    # Initialize state estimation process
    state_estimation_process = mp.Process(target=state_estimation_control_process, args=(start_flag_mp, timeout_flag_mp, aeolus.lane_queue, aeolus.teensy_state_queue, aeolus.state_queue, aeolus.speed_signal, aeolus.str_angle, aeolus.sm, log_queue))

    logger.info("AUTONOMY MAIN: Finished initializing processes. Starting threads")
    # Start the measurement threads
    teensy_state_reader_thread.start()
    camera_capture_thread.start()
    logger.info("AUTONOMY MAIN: Finisished starting threads. Starting processes")
    logger.info("AUTONOMY MAIN: Starting processes")
    neural_net_lane_detection_process_instance.start()
    state_estimation_process.start()
    logger.info("AUTONOMY MAIN: Finished starting processes")

    # End all the threads
    teensy_state_reader_thread.join()
    camera_capture_thread.join()

    # End all the processes
    neural_net_lane_detection_process_instance.join()
    state_estimation_process.join()

    listener.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="The main for the aeolus autonomous vehicle")
    parser.add_argument('-r', '--record', action='store_true')
    parser.add_argument('-s', '--spoofing', action='store_true')
    args = parser.parse_args()

    # Empty the torch cache
    torch.cuda.empty_cache()
    # Set the process start method to fork (for CUDA compatibility)
    mp.set_start_method("spawn", force = True)

    if not args.record:
        if args.spoofing:
            run_with_spoofing = True
        else:
            run_with_spoofing = False
        autonomy(speed_matrix=HARD_SPEED_MATRIX, spoofing = run_with_spoofing, recording_path = hard_recording_path)
    else:
        record(speed_matrix=HARD_SPEED_MATRIX)
