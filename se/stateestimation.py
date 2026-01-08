import os
import struct
import math
from datetime import datetime
import numpy as np

from core.cv.nueral_net.vision_pipeline.lane_detection import Lane
from .EKF import BicycleEKF

from core.constants import LOG_DIR, OUTPUT_IMAGE_WIDTH, OUTPUT_IMAGE_HEIGHT, TEENSY_SAMPLE_PERIOD_THRESHOLD, CAMERA_GRID_Y_OFFSET, PPM, V_LENGTH, CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, BEV_FOV_WIDTH, BEV_FOV_HEIGHT, STATE_PACKER_FORMAT
import logging

logger =  logging.getLogger(__name__)

#Decription: The vehicle state class and the methods required to get each state element. Uses the latest image from the lane detection pipeline and the latest sensor data.

class VehicleState:
    """
    Digital representation of the vehicle state
    """
    
    def __init__(self, vehicle, control_covar, measurement_covar):
        """
        Constructor

        param img: The birds eye view image from lane detection algorithm
        param fit: The path polynomial fit from lane detection algorithm
        """
        ###--------------------///Computer Representation Variables///--------------------###
        ###------------------------------///Vehicle Parameters///-----------------------------###
        self.wrad = vehicle.wheel_rad
        self.a2a_length = vehicle.a2a_length #  The axel to axel length of the vehicle
        self.voffset = vehicle.ca_offset # The length from the camera to the rear axel
        self.vlength = vehicle.length # The length of the vehicle
        self.vwidth = vehicle.width # The width of the vehicle
        self.cam2im = vehicle.cam2im # The distance between the camera and where the camera plane intersects the track

        #--------------Filter Objects--------------#
        self.control_covar = control_covar
        self.measurement_covar = measurement_covar
        self.EKF = BicycleEKF(control_covar, measurement_covar, vehicle)
        
        ###------------------------------////State Variables///-------------------------------###
        #----------Time Vars----------#
        self.timestamp = None
        self.image_timestamp = None

        #----------Image Frame State Vars----------#
        # The position and heading of the vehicle in the image frame
        self.lcl_x = None
        self.lcl_y = None
        self.lcl_theta = None

        #----------Track Global Frame State Vars----------#
        # The position and heading of the vehicle in the global coordinate frame
        self.glb_x = None
        self.glb_y = None
        self.glb_heading = None

        self.EKF_covar = None

        #----------Environmental Variables----------#
        self.path_y = None # The path y coordinates in the image frame in meters
        self.path_x = None # The path x coordinates in the vehicle frame in meters

        #----------Runnning Variables---------#
        # The total distance traveled by the vehicle
        self.dist = None

        #----------Dynamic State Variables----------#
        # The speed of the vehicle
        self.velocity = None

        # The steering angle of the vehicle
        self.steer_angle = None

        # The acceleration of the vehicle 
        self.accel_x = None
        self.accel_y = None

        if ((self.accel_x is not None) & (self.accel_y is not None)):  #TODO: Add in microcontroller code instead
            self.accel = np.sqrt(self.accel_x**2 + self.accel_y**2)

        self.ultra_distance = None

        #----------Path Relation Variables----------#
        # The lateral displacement from the reference point of the vehicle to the lane (perpendicular to the vehicle heading)
        self.lat_disp = None

        # The lateral displacement from the path to the reference point of the vehicle (perpendicular to the path)
        self.lat_pdisp = None

        # The cross-track error of the vehicle and lane
        self.cterror = None

        #----------State Saving Variables-----------#
        self.state_packer_string = STATE_PACKER_FORMAT
        self.state_store = None
        self.state_packer = None

    def initialize_state(self, first_lane):
        """
        Initializes the state based on an initialized image and lane. Asssumes zero velocity

        param first_lane: The first processed lane lines
        """
        logger.info("STATE ESTIMATION: Initializing first state")
        # Initialize timestamps
        self.timestamp = datetime.now()
        self.image_timestamp = first_lane.timestamp

        # Initialize running variables
        self.dist = 0

        # Initialize global variables
        self.glb_x = 0
        self.glb_y = 0
        self.glb_heading = 0

        # Initialize all of the image state variables
        self.lcl_x = 0
        self.lcl_y = 0
        self.lcl_theta = np.pi / 2

        self.path_x = first_lane.path_x
        self.path_y = first_lane.path_y

        # Set all the instantaneous state variables
        self.velocity = 0
        self.accel_x = 0
        self.accel_y = 0
        self.steer_angle = 0

        # Set teensy variables
        self.ultra_distance = 400

    def start_logging(self):
        """
        Starts binary logging of states
        """
        # Open the state store file
        state_store_name = f"state_logs_{self.timestamp.strftime('%H-%M-%S')}"
        state_store_path = os.path.join(LOG_DIR, state_store_name)
        self.state_store = open(state_store_path, "wb", buffering=0)
        self.state_packer = struct.Struct(self.state_packer_string)

    def save_state(self):
        """
        Writes the current state to a binary record
        """

        ts_nanoseconds = self.timestamp.timestamp() * 1e9
        ts_nanoseconds = int(ts_nanoseconds)

        image_ts_nanoseconds = self.image_timestamp.timestamp() * 1e9
        image_ts_nanoseconds = int(image_ts_nanoseconds)
        
        state_record = self.state_packer.pack(ts_nanoseconds, image_ts_nanoseconds, self.lcl_x, self.lcl_y, self.lcl_theta, self.glb_x, self.glb_y, self.glb_heading,
                                            self.dist, self.velocity, self.steer_angle, self.accel_x, self.accel_y, self.ultra_distance)
        
        self.state_store.write(state_record)

    #-----------Motion Model Based State Updater-----------#
    def propagate_bicycle_state(self, prev_state, v, delta, dt):
        """
        Propagates the six bicycle motion model state vectors using the bicycle motion model

        prev_state: The previous bicycle state vector
        v: The current velocitry (m/s)
        delta: The current steering angle (rad)
        dt: The time delta between the current timestamp and the previous state (s)

        returns: The updated bicycle state vector
        """
        # Decant state
        x_prev = prev_state[0]
        y_prev = prev_state[1]
        theta_prev = prev_state[2]

        logger.debug(f"STATE ESTIMATION PROPAGRATION: Propagating using x_p: {x_prev}, y_p: {y_prev}, theta_p: {theta_prev}, delta: {delta}, v: {v}, dt: {dt}")
        
        # Propogate state according to motion model
        w = (v * math.tan(delta)) / self.a2a_length
        theta = theta_prev + w * dt
        vx = v * math.cos(theta)
        vy = v * math.sin(theta)
        x = x_prev + vx * dt
        y = y_prev + vy * dt

        logger.debug(f"STATE ESTIMATION PROPAGRATION: Calculted bicycle model parameters: {w}, theta {theta}, vx: {vx}, vy: {vy}")
        logger.debug(f"STATE ESTIMATION PROPAGATION: Returning new state, x: {x}, y: {y}, theta: {theta}")

        return np.array([x, y, theta, vx, vy, w])
    
    def update_delta_array(self, delta_array, timestamp, img_state):
        """
        Updates the delta array for a given timestep

        timestamp: The timestamp for the current timestep
        delta_array: An array of the deltas (x, y, theta) for the last n timesteps
        """
        # Construct a list of timestamp, delta_x, delta_y, theta
        current_deltas = np.array([timestamp, img_state[0], img_state[1], img_state[2]])
        delta_array.append(current_deltas)

    def propagate_with_delta_array(self, delta_array, image_timestamp):
        """
        Uses the delta array to propagate an image from the time it was taken to now
        """
        dx_prime = 0
        dy_prime = 0
        delta_theta = 0

        if delta_array:
            check_delta = delta_array.popleft()

            # Iterate through until you reach the first teensy state after image capture
            while delta_array and check_delta[0] < image_timestamp:
                check_delta = delta_array.popleft()
                logger.warning(f"STATE ESIMATION: Popping delta array from timestamp: {check_delta[0]} from before capture time: {image_timestamp}")

            # Record the local x, y, and theta at the time of image capture (approximated as first state after capture TODO: Look into interpolation between state before capture and state after)
            old_dx = check_delta[1]
            old_dy = check_delta[2]
            old_theta = check_delta[3] 
            rot_theta = old_theta - np.pi/2
            logger.warning(f"STATE ESTIMATION: Progating with delta array, old image state at capture time x:{round(old_dx, 4)}, y:{round(old_dy, 4)}, old_theta:{round(old_theta, 4)}")

            # Loop till the most recent state
            while delta_array:
                check_delta = delta_array.popleft()
                logger.warning(f"STATE ESIMATION: Popping delta array from timestamp: {check_delta[0]} after capture time {image_timestamp}")

            # Get the local x, y, and theta at the final recorded teensy state
            dx_final = check_delta[1]
            dy_final = check_delta[2]
            theta_final = check_delta[3]
            logger.warning(f"STATE ESTIMATION: Progating with delta array, final old image state x:{round(dx_final, 4)}, y:{round(dy_final, 4)}, old_theta:{round(theta_final, 4)}")

            new_dx = dx_final - old_dx
            new_dy = dy_final - old_dy

            logger.warning(f"STATE ESTIMATION: Progating with delta array, new delta x:{round(new_dx, 4)}, y:{round(new_dy, 4)}")

            # Rotate the deltas into the frame of the new 
            dx_prime = new_dx*np.cos(rot_theta) - new_dy*np.sin(rot_theta)
            dy_prime = new_dx*np.sin(rot_theta) + new_dy*np.cos(rot_theta)

            # Get the angle offset betweeen the final state and the image capture state
            delta_theta = theta_final - old_theta

        return dx_prime, dy_prime, delta_theta

    def update_to_new_image(self, new_lane_timestamp, delta_array, teensy_state_queue, steering_angle):
        """
        Updates the state vector given new lane information from an image and measurements from the teensy

        new_lane_timestamp (datetime): The timestamp from when the latest lane detection image was captured
        delta_array: An array of t he deltas (x, y, theta) for the last n timesteps
        teensy_state_queue (Queue[TeensyState]): A queue of recently collected teensy measurements
        steering_angle (float): The most recent steering angle command sent to the teensy (rad)

        return (img_state, i_teensy_state): The new image state and the most recent teensy state vector
        """
        logger.debug("STATE ESTIMATION: Updating state to new image")

        # Set the current image timestamp
        self.image_timestamp = new_lane_timestamp

        # Propagate the local state from when the image was taken to the current time using the delta array
        dx, dy, d_theta = self.propagate_with_delta_array(delta_array, new_lane_timestamp)

        # Set the new image state based on propagation
        img_state = [dx, dy, d_theta + np.pi/2, 0, 0, 0] 
        logger.warning(f"STATE ESTIMATION: Propagated within NEW image using delta array x:{img_state[0]} y:{img_state[1]}, theta:{img_state[2]}")

        prev_teensy_timestamp = self.timestamp

        # Loop through the remaining states and use them to propagate the vehicles coordinates within the image frame
        while not teensy_state_queue.empty():
            # Get the next state
            i_teensy_state = teensy_state_queue.get()

            # Get all the variables required to propagate state
            v = i_teensy_state.velocity_mag
            delta = steering_angle
            dt = (i_teensy_state.timestamp - prev_teensy_timestamp).total_seconds()

            # Throw an error message if dt is very large
            if dt > TEENSY_SAMPLE_PERIOD_THRESHOLD:
                logger.warning(f"STATE ESTIMATION: Very large time gap between teensy state samples detected ({dt}s)")

            logger.warning(f"STATE ESTIMATION: Propagating within NEW image (image_time: {new_lane_timestamp}) at t {i_teensy_state.timestamp} using v: {round(v, 3)}m/s, delta: {round(delta, 3)}rad, dt: {round(dt, 4)}s")

            # Propagate the vehicles coordinate within the image frame
            img_state = self.propagate_bicycle_state(img_state, v, delta, dt)

            # Update the total distance
            self.dist += v * dt

            # Set the previous variables for the next go around
            prev_teensy_timestamp = i_teensy_state.timestamp

        logger.info(f"STATE ESTIMATION: Propagated in NEW image to: x:{img_state[0]} y:{img_state[1]}, theta:{img_state[2]}")
        logger.info(f"STATE ESTIMATION: New distance: {self.dist}m")
                
        return img_state, i_teensy_state
    
    def propagate_in_current_image(self, previous_local_image_state, delta_array, teensy_state_queue, steering_angle):
        """
        Updates the state vector within the most recent images local frame given new teensy measurements

        previous_local_image_state: The local image variables from the most recently processed image
        delta_array: An array of the deltas (x, y, theta) for the last n timesteps
        teensy_state_queue (Queue[TeensyState]): A queue of recently collected teensy measurements
        steering_angle (float): The most recent steering angle command sent to the teensy

        return (img_state, i_teensy_state): The propagated image state and the most recent teensy state vector
        """
        logger.debug("STATE ESTIMATION: Propagating state within current image")

        img_state = previous_local_image_state
        prev_teensy_timestamp = self.timestamp

        while not teensy_state_queue.empty():
            # Get the next state
            i_teensy_state = teensy_state_queue.get()

            # Get all the variables required to propagate state
            v = i_teensy_state.velocity_mag
            delta = steering_angle
            dt = (i_teensy_state.timestamp - prev_teensy_timestamp).total_seconds()

            # Throw an error message if dt is very large
            if dt > TEENSY_SAMPLE_PERIOD_THRESHOLD:
                logger.debug(f"STATE ESTIMATION: Very large time gap between teensy state samples detected ({dt}s)")

            logger.warning(f"STATE ESTIMATION: Propagating within OLD image (last state time: {prev_teensy_timestamp}) at t {i_teensy_state.timestamp} using v: {round(v, 3)}m/s, delta: {round(delta, 3)}rad, dt: {round(dt, 4)}s")

            # Propagate the vehicles coordinate within the image frame
            img_state = self.propagate_bicycle_state(img_state, v, delta, dt)

            logger.warning(f"STATE ESTIMATION: Propagating within OLD image to x:{img_state[0]} y:{img_state[1]}, theta:{img_state[2]}")

            # Update the delta array using this state
            self.update_delta_array(delta_array, i_teensy_state.timestamp, img_state)

            # Update the total distance
            self.dist += v * dt

            prev_teensy_timestamp = i_teensy_state.timestamp

        logger.info(f"STATE ESTIMATION: New distance: {self.dist}m")

        return img_state, i_teensy_state

    #----------Derived State Variable Calculators----------#
    def calc_lcl_heading(self):
        """
        Calculates the image local heading
        """
        return self.lcl_theta - np.pi / 2
    
    def calc_pdistances(self):
        """
        Calculates the distance from the path to the vehicle refrence point at each point on the path
        """

    def calc_angerror(self):
        """
        Calculates the error between the path heading at the point on the path closest to the vehicle (as measured by lines perpendicular to the path) and the vehicle heading
        """

    def calc_cterror(self):
        """
        Calculates the cross track error
        """

    def calc_lat_disp(self, pathx, pathy):
        """
        Calculate the displacement (perpendicular to the vehicle) from the vehicle refrence point to the path 

        return: The displacement from the vehicle refrence point to the path
        """
        lat_disp = pathx(np.where(pathy == 0))
        self.lat_disp = lat_disp

        return lat_disp

    def calc_lat_pdisp(self, pathx, pathy):
        """
        Calculate the displacement (perpendicular to the path) from the path to the vehicle reference point

        return: The displacement from the path to the vehicle reference point
        """

    def update_state(self, new_lane, new_teensy_states, delta_array, steering_angle_mp):
        """
        Update all the state variables

        param new_lane (Lane): The most fresh lane object from the nueral net/lane detection pipeline
        param new_teensy_states (Queue[TeensyState]): A queue of all teensy states recieved since last state estimation cycle
        param delta_array: An array of the deltas (x, y, theta) for the last n timesteps
        param steering_angle_mp: The multiprocessing value storing the current steering angle
        """
        logger.info("STATE ESTIMATION: Starting state update")

        steering_angle = steering_angle_mp.value

        # State update in the case where there is a new lane available
        if (new_lane is not None):
            logger.info("STATE ESTIMATION: Updating state from new image")

            # Reset all of the image state variables
            self.lcl_x = 0
            self.lcl_y = 0
            self.lcl_theta = np.pi / 2

            # Unpack the current lane object
            self.path_x = new_lane.path_x
            self.path_y = new_lane.path_y

            # Case where we have new teensy state measurements
            if not new_teensy_states.empty():
                # Propogate the new image coordinates up to the current time using the input stack
                img_state, current_teensy_state = self.update_to_new_image(new_lane.timestamp, delta_array, new_teensy_states, steering_angle)

                # Set all the image state variables
                self.lcl_x = img_state[0]
                self.lcl_y = img_state[1]
                self.lcl_theta = img_state[2]

                # Set all the instantaneous teensy state variables
                self.velocity = current_teensy_state.velocity_mag
                self.accel_x = current_teensy_state.x_acc
                self.accel_y = current_teensy_state.y_acc
                self.ultra_distance = current_teensy_state.ultra_dist / 100 # Convert from cm to m
                self.steer_angle = steering_angle
            
            # If we don't have any new teensy state variables, keep the state as initialized
            else:
                logger.warning("STATE ESTIMATION: New lane detection information but no new teensy state information")

        # State update in the case where we don't have any new lane information
        else:
            # Case where we have new teensy state measurements
            if not new_teensy_states.empty():
                logger.info("STATE ESTIMATION: Propagating state within old image using teensy measurements")
                previous_image_state = (self.lcl_x,  self.lcl_y, self.lcl_theta)
                img_state, current_teensy_state = self.propagate_in_current_image(previous_image_state, delta_array, new_teensy_states, steering_angle)

                # Set all the image state variables
                self.lcl_x = img_state[0]
                self.lcl_y = img_state[1]
                self.lcl_theta = img_state[2]

                # Set all the instantaneous teensy state variables
                self.velocity = current_teensy_state.velocity_mag
                self.accel_x = current_teensy_state.x_acc
                self.accel_y = current_teensy_state.y_acc
                self.ultra_distance = current_teensy_state.ultra_dist / 100 # Convert from cm to m
                self.steer_angle = steering_angle
              
            # If we don't have any new teensy state variables, keep the state as initialized
            else:
                logger.warning("STATE ESTIMATION: No new lane information and no new teensy measurements detected")

        # Set the timestamp to now
        self.timestamp = current_teensy_state.timestamp
        """
        # TODO: Unquarrentine if GPS is implemented
        # Updating the state in the case that there are new gps values available
        if (new_gps is not None):
            # Call the EKF to get the new state
            p_state = np.array([self.glb_x, self.glb_y, self.glb_heading])
            control_input = np.array([c_vel, self.steer_angle])
            measurement = np.array([gps_x, gps_y, c_heading])
            self.EKF.EKF_state_update(p_state, self.EKF_covar, control_input, measurement, dt)
        """