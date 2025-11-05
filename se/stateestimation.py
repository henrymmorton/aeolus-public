import os
import cv2
import math
from datetime import datetime
from queue import LifoQueue
import numpy as np
import matplotlib.pyplot as plt

from core.cv.nueral_net.vision_pipeline.lane_detection import Lane
from .EKF import BicycleEKF

from core.cv.nueral_net.vision_pipeline.perspective_transform_measured import CameraPerspectiveTransform
from core.constants import LOG_DIR, OUTPUT_IMAGE_WIDTH, OUTPUT_IMAGE_HEIGHT, TEENSY_SAMPLE_PERIOD_THRESHOLD, CAMERA_GRID_Y_OFFSET, PPM, V_LENGTH, CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, BEV_FOV_WIDTH, BEV_FOV_HEIGHT
import logging

logger =  logging.getLogger(__name__)

#Decription: The vehicle state class and the methods required to get each state element. Uses the latest image from the lane detection pipeline and the latest sensor data.

class VehicleState:
    """
    Digital representation of the vehicle state
    """
    
    def __init__(self, vehicle, control_covar, measurement_covar):
        """Default constructor

        param img: The birds eye view image from lane detection algorithm
        param fit: The path polynomial fit from lane detection algorithm
        """
        ###--------------------///Computer Representation Variables///--------------------###
        #----------Image Data----------#
        self.img = None
        self.virtual_img = None

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

    def calc_lcl_heading(self):
        """
        Gets the image local heading
        """
        return self.lcl_theta - np.pi / 2

    #-----------Motion Model Based State Updater-----------#shou
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
    
    def update_to_new_image(self, new_lane_timestamp, teensy_state_queue, steering_angle):
        """
        Updates the state vector given new lane information from an image and measurements from the teensy

        new_lane_timestamp (datetime): The timestamp from when the latest lane detection image was captured
        teensy_state_queue (Queue[TeensyState]): A queue of recently collected teensy measurements
        steering_angle (float): The most recent steering angle command sent to the teensy (rad)

        return (img_state, i_teensy_state): The new image state and the most recent teensy state vector
        """
        logger.debug("STATE ESTIMATION: Updating state to new image")

        # Initialize the new image state
        img_state = [0, 0, np.pi/2, 0, 0, 0] 

        if not teensy_state_queue.empty():
            i_teensy_state = teensy_state_queue.get()
            prev_teensy_timestamp = i_teensy_state.timestamp

            # Loop through the state queue untill you reach the first measurement taken after the image
            while (i_teensy_state.timestamp <= new_lane_timestamp) and (not teensy_state_queue.empty()):
                i_teensy_state = teensy_state_queue.get()
                prev_teensy_timestamp = i_teensy_state.timestamp
                logger.info(f"STATE ESTIMATION: Skipped teensy state from {i_teensy_state.timestamp} before image capture time: {new_lane_timestamp}")

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

                logger.info(f"STATE ESTIMATION: Propagating within NEW image (image_time: {new_lane_timestamp}) at t {i_teensy_state.timestamp} using v: {round(v, 3)}m/s, delta: {round(delta, 3)}rad, dt: {round(dt, 4)}s")

                # Propagate the vehicles coordinate within the image frame
                img_state = self.propagate_bicycle_state(img_state, v, delta, dt)

                # Update the total distance
                self.dist += v * dt

                # Set the previous variables for the next go around
                prev_teensy_timestamp = i_teensy_state.timestamp
                
        return img_state, i_teensy_state
    
    def propagate_in_current_image(self, previous_local_image_state, teensy_state_queue, steering_angle):
        """
        Updates the state vector within the most recent images local frame given new teensy measurements

        previous_local_image_state: The local image variables from the most recently processed image
        teensy_state_queue (Queue[TeensyState]): A queue of recently collected teensy measurements
        steering_angle (float): The most recent steering angle command sent to the teensy

        return (img_state, i_teensy_state): The propagated image state and the most recent teensy state vector
        """
        logger.debug("STATE ESTIMATION: Propegating state within current image")

        img_state = previous_local_image_state

        if not teensy_state_queue.empty():
            i_teensy_state = teensy_state_queue.get()
            prev_teensy_timestamp = i_teensy_state.timestamp

            prev_state_timestamp = self.timestamp

            # Loop through the state queue untill you reach the first measurement taken after the last state estimation run
            while (i_teensy_state.timestamp <= prev_state_timestamp) and (not teensy_state_queue.empty()):
                i_teensy_state = teensy_state_queue.get()
                prev_teensy_timestamp = i_teensy_state.timestamp
                logger.info(f"Skipped teensy state from: {i_teensy_state.timestamp} before last state estimation run time: {prev_state_timestamp}")

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

                logger.info(f"STATE ESTIMATION: Propagating within OLD image (last state time: {prev_state_timestamp}) at t {i_teensy_state.timestamp} using v: {round(v, 3)}m/s, delta: {round(delta, 3)}rad, dt: {round(dt, 4)}s")

                # Propagate the vehicles coordinate within the image frame
                img_state = self.propagate_bicycle_state(img_state, v, delta, dt)

                # Update the total distance
                self.dist += v * dt

                prev_teensy_timestamp = i_teensy_state.timestamp

            return img_state, i_teensy_state

    #----------Plotting Functions----------# 
    def plot_vehicle(self, new_lane):
        """
        Adds two vehicle lengths of "virtual space" behind the camera and extends the path into the space according to the fit

        param img: The perspective warped image
        param fit: The polynomial fit coefficients for the path
        """
        
        if new_lane:
            if self.img is None:
                logger.warning(f"STATE ESTIMATION PLOTTING: No image passed to state estimation")
                return
            # Plot the last lane image
            logger.warning(f"STATE ESTIMATION PLOTTING: New image available for plotting")
            figure, axis1 = plt.subplots(1,1)
            axis1.imshow(self.virtual_img)
            # TODO: Determine how to show path too
            axis1.set_title("Warped Image with Virtual Space")
            save = True
            if save:
                plt.savefig(os.path.join(LOG_DIR, f"vehicle_{round(self.dist, 4)}.png"))
            else:
                plt.show()

            # Zero out the image and add the new one
            self.virtual_img[:] = 0
            self.virtual_img[0:self.img.shape[0], 0:self.img.shape[1], :] = self.img
            self.virtual_img = cv2.cvtColor(self.virtual_img.astype('uint8'), cv2.COLOR_BGR2RGB)

        # Draw the vehicle as a rectangle
        # TODO: Offset this from the vehicle origin to the center of the vehicle
        logger.warning (f"STATE ESTIMATION PLOTTING: Rectangle params CX: {self.lcl_x}, CY: {self.lcl_y}, W:{self.vwidth}, L:{self.vlength}, HEADING:{self.calc_lcl_heading()}")
        vehicle_rectangle_params = ((self.lcl_x, self.lcl_y), (self.vwidth, self.vlength), np.rad2deg(self.calc_lcl_heading()))
        vehicle_rectangle = cv2.boxPoints(vehicle_rectangle_params)      # Returns float32 array

        logger.info(f"STATE ESTIMATION PLOTTING: Vehicle rectangle meters corners {vehicle_rectangle}")

        # TODO: Transform this into pixel space
        perspective_transformer = CameraPerspectiveTransform(CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, PPM, BEV_FOV_WIDTH, BEV_FOV_HEIGHT, CAMERA_GRID_Y_OFFSET)
        vehicle_rectangle_pixels = perspective_transformer.vehicle2image_transform(vehicle_rectangle)
        vehicle_rectangle_pixels_BEV = perspective_transformer.image2BEVimage_transform(vehicle_rectangle_pixels).astype(np.int32)

        logger.info(f"STATE ESTIMATION PLOTTING: Vehicle rectangle pixel corners {vehicle_rectangle_pixels_BEV}")

        cv2.polylines(self.virtual_img, [vehicle_rectangle_pixels_BEV], isClosed=True, color=(0, 255, 0),  thickness=5)

        plt.close()

    #----------Derived State Variable Calculators----------#
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

    def initializeState(self, first_lane, plot):
        """
        Initialize the state based on an initialized image and lane. Asssumes zero velocity

        """
        logger.info("STATE ESTIMATION: Initializing first state")
        # Initialize timestamp
        self.timestamp = datetime.now()

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

        if plot:
            # Create a expanded image to hold the birds eye view
            # Calculate how much y space it needed to include the vehcile and some buffer
            additional_space = CAMERA_GRID_Y_OFFSET*PPM + V_LENGTH*PPM
            new_length = np.rint(first_lane.birds_eye_image.shape[0] + additional_space).astype(int)

            # Create an image that includes the "virtual space"
            self.virtual_img = np.empty((new_length, first_lane.birds_eye_image.shape[1], 3))

    def updateState(self, new_lane, new_teensy_states, steering_angle_mp, plot = False):
        """
        Update all the state variables

        param new_lane (Lane): The most fresh lane object from the nueral net/lane detection pipeline
        param new_teensy_states (Queue[TeensyState]): A queue of all teensy states recieved since last state estimation cycle
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
            self.img = new_lane.birds_eye_image
            self.path_x = new_lane.path_x
            self.path_y = new_lane.path_y

            # Case where we have new teensy state measurements
            if not new_teensy_states.empty():
                # Propogate the new image coordinates up to the current time using the input stack
                img_state, current_teensy_state = self.update_to_new_image(new_lane.timestamp, new_teensy_states, steering_angle)

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
                img_state, current_teensy_state = self.propagate_in_current_image(previous_image_state, new_teensy_states, steering_angle)

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

        if plot:
            if (new_lane is not None):
                self.plot_vehicle(new_lane=True)
            else:
                self.plot_vehicle(new_lane=False)

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