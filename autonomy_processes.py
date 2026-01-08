import os
import time
import logging
from datetime import datetime
import psutil
from collections import deque
import torch.multiprocessing as mp

from .cv.nueral_net.vision_pipeline.ERFNetPredictor import ERFNetPredictor
from .cv.nueral_net.vision_pipeline.lane_detection import Lane, lane_detection_pipeline
from .sc.purepursuit import PurePursuit, pure_pursuit_pipeline
from .sc.filters import LowPassFilter
from .sc.obstacle_handling import ObstacleHandler

from .constants import STATE_ESTIMATION_LOOP_PERIOD, STEERING_FILTER_TAU

from .logging_config import PROCESS_LOGGING_CONFIG, PROCESS_LOGGING_CONFIG_OWN_FILE
logging.config.dictConfig(PROCESS_LOGGING_CONFIG)

def neural_net_lane_detection_process(timeout_flag, image_queue, lane_queue, model_path, perspective_transformer, log_queue):
    """
    Perform nueral net image segmentation and lane detection on images output by the camera

    param timeout_flag: A multiprocessing event signaling timeout
    param image_queue: A multiprocessing queue of images captured by the camera (process fetches from)
    param lane_queue: A multiprocessing queue of detected lane objects (process pushes to)
    param model_path: A path to the pytorch model script (TorchScript)
    param perspective_transformer: A CameraPerspectiveTransform object with methods for moving between pixel and meter reference frames
    param log_queue: A multiprocessing queue for collecting logs from this process
    """
    # Allocate 3 CPU cores to this process
    nn_pid = os.getpid()
    p = psutil.Process(nn_pid)
    p.cpu_affinity([2, 3, 4])

    # Setup logging for this process
    logger = logging.getLogger("core")
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.warning(f"Nueral Net/Lane Detection PID: {nn_pid}")

    # Create an instance of the model (this has to be done inside the process for some reason)
    predictor = ERFNetPredictor(model_path)

    nn_loop_count = 0
    
    inference_time_sum = 0
    inference_average_period = 0
    inference_average_freq = 0

    process_time_sum = 0
    process_average_period = 0
    process_average_freq = 0

    while not timeout_flag.is_set():
        logger.info(f"NEURAL NET/LANE DETECTION PROCESS STATUS: Starting process, Loop count {nn_loop_count}, Image queue status: {image_queue.empty()}")
        if not image_queue.empty():

            p_start = time.time()

            # Unpack the image and timestamp
            current_frame_plus_time = image_queue.get()
            frame_time = current_frame_plus_time[1]

            # Call the neural net predictor
            current_inference = predictor.predict_lane_lines(current_frame_plus_time[0])

            i_end = time.time()

            # Call the lane detection pipeline
            lane_object = lane_detection_pipeline(perspective_transformer, current_inference, frame_time, image=current_frame_plus_time[0])

            # Put the lane image in the queue
            if lane_object is not None:
                # If the queue is full, remove the oldest lane
                if lane_queue.full():
                    lane_queue.get()

                lane_queue.put(lane_object)

            # Update benchmarks
            nn_loop_count += 1
            p_end = time.time()
            i_duration = i_end - p_start
            p_duration = p_end - p_start
            inference_time_sum += i_duration
            process_time_sum += p_duration
            inference_average_period = inference_time_sum / nn_loop_count
            process_average_period = process_time_sum / nn_loop_count
            inference_average_freq = (1 / inference_average_period)
            process_average_freq = (1 / process_average_period)

            if nn_loop_count % 50 == 0:
                logger.debug(f"NUERAL NET/LANE DETECTION PROCESS STATUS: Average inference freq/period: {round(inference_average_freq, 5)}Hz, {round(inference_average_period, 5)}s, Average process freq/period: {round(process_average_freq, 5)}Hz, {round(process_average_period, 5)}s")

        else:
            logger.warning("NEURAL NET/LANE DETECTION PROCESS STATUS: Image queue empty")
            time.sleep(1/10)

        logger.info(f"NEURAL NET/LANE DETECTION PROCESS STATUS: Finished process")

                       
def state_estimation_control_process(start_flag, timeout_flag, lane_queue, teensy_state_queue, vehicle_state_queue, mp_speed, mp_steering_angle, speed_matrix, log_queue):
    """
    Estimate the vehicle state using the detected lane information and teensy state. Use state to calculate control signals and send to teensy

    param start_flag: A multiprocessing event cued by the start of the SE/C process
    param timeout_flag: A multiprocessing event signaling timeout
    param lane_queue: A multiprocessing queue of detected lane objects (process pulls from)
    param teensy_state_queue: A multiprocessing queue of teensy_state fetched from teensy via serial (process pulls from)
    param vehicle_state_queue: A queue of vehicle states used to store the intial state
    param mp_speed: The multiprocessing value containing the current speed (commanded not actual)
    param mp_steering_angle" The multiprocessing value containing the current steering angle
    param speed_matrix: The matrix of times and speeds which dictates vehicle speed in the run
    param log_queue: A multiprocessing queue for collecting logs from this process
    """
    # Allocate 1 CPU core to this process
    sec_pid = os.getpid()
    p = psutil.Process(sec_pid)
    p.cpu_affinity([1])

    # Setup logging for this process
    logger = logging.getLogger("core")
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.warning(f"State Estimation/Control PID: {sec_pid}")

    pursuit = PurePursuit()
    steering_low_pass = LowPassFilter(loop_rate=STATE_ESTIMATION_LOOP_PERIOD, time_constant=STEERING_FILTER_TAU)
    obstacle_handler = ObstacleHandler()

    # Start the timer for the speed matrix
    state_estimation_startup_time = datetime.now()
    speed_matrix.set_startup_time(state_estimation_startup_time)

    vehicle_state = vehicle_state_queue.get()
    vehicle_state.start_logging()
    vehicle_state.save_state()

    delta_array = deque(maxlen=30)

    se_loop_count = 0

    start_flag.set()

    while not timeout_flag.is_set():
        logger.info(f"STATE ESTIMATION/CONTROL PROCESS STATUS: Process started, Queue statuses, Lane: {lane_queue.empty()}, Teensy: {teensy_state_queue.empty()}")

        # Case where a new lane object is availabile and new teensy measurements are available
        if (not lane_queue.empty()) and (not teensy_state_queue.empty()):
            # Get the most recent lane detection information
            while not lane_queue.empty():
                new_lane_obj = lane_queue.get()

            vehicle_state.update_state(new_lane_obj, teensy_state_queue, delta_array, mp_steering_angle)
            time.sleep(1/200)
        
        # Case where new teensy measurements are available but no new lane object are available
        elif (lane_queue.empty()) and (not teensy_state_queue.empty()):
            vehicle_state.update_state(None, teensy_state_queue, delta_array, mp_steering_angle)

        # Case where a new lane object is available but no new teensy measurements are available
        elif (not lane_queue.empty()) and (teensy_state_queue.empty()):
            logger.warning("STATE ESTIMATION/CONTROL PROCESS STATUS: A new lane object is available but there are no teensy states. Sleeping for 0.01s")
            time.sleep(1/100)

        # Case where niether a new lane object or new teensy measurements are available
        else:
            logger.debug("STATE ESTIMATION/CONTROL PROCESS STATUS: There are no new teensy states or lane objects available. Sleeping for 0.01s")
            time.sleep(1/100)

        # Save the state to disk
        vehicle_state.save_state()
        # Use pure pursuit to determine the control signals to send to the teensy
        new_steering_angle = pure_pursuit_pipeline(vehicle_state, pursuit)
        # Pass the steering value through a low pass filter
        filtered_new_steering_angle = steering_low_pass.update(new_steering_angle)
        # Set the multiprocessing value to the filtered value
        mp_steering_angle.value = filtered_new_steering_angle

        # Get the currently requested speed and update the multiprocessing speed value
        control_speed, stop_flag = speed_matrix.get_current_speed()
        mp_speed.value = control_speed
        logger.debug(f"STATE ESTIMATION/CONTROL PROCESS STATUS: Computed control outputs [speed: {control_speed}m/s, steering: {new_steering_angle}rad]")
        
        obstacle_stop = obstacle_handler.update_handler(vehicle_state)
        
        # Check to see if the timeout flag has been triggered
        if stop_flag:
            logger.error("STATE ESTIMATION/CONTROL PROCESS STATUS: Timeout flag set")
            timeout_flag.set()

        se_loop_count += 1
        logger.info("STATE ESTIMATION/CONTROL PROCESS STATUS: Process finished")