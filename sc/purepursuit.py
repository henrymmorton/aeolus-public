import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from core.constants import LOOKAHEAD_GAIN, MINIMUM_LOOKAHEAD_DISTANCE, A2A_LENGTH
import logging

logger =  logging.getLogger(__name__)

class PurePursuit:
    """
    An implementation of the pure pursuit algorithm
    """

    def __init__(self, lookahead_gain = LOOKAHEAD_GAIN, minimum_lookahead = MINIMUM_LOOKAHEAD_DISTANCE, vehicle_length = A2A_LENGTH):
        """
        Default constructor

        param lookahead_gain: A constant that scales how far the lookahead distance is
        param minimum_lookahead: A constant representing the minimum lookahead distance (m)
        param vehicle_length: The axel to axel length of the vehicle (m)
        """
        #----------Pure Pursuit Parameters----------#
        self.k_lkahead = lookahead_gain
        self.minimum_lookahead = minimum_lookahead

        #-----------Vehicle Parameters--------------#
        self.vehicle_length = vehicle_length
    
    def frame_transform(self, x_pic, y_pic, T_x, T_y, theta):
        """
        Transforms the coordinate from the image local frame (origin at bottom center of the image)
        to vehicle local frame (origin at center of back axel of wheel)

        param x_pic: A numpy array of x coordinates in the image local frame
        param y_pic: A numpy array of y coordinates in the image local frame
        param T_x: The x coordinate of the vehicle in the image local frame
        param T_y: The y coordinate of the vehicle in the image local frame
        param theta: The angular offset between the image local and vehicle local frames (rad)
        """
        # Translate to the vehicle local frame
        dx = x_pic - T_x
        dy = y_pic - T_y

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        x_v = dx * cos_theta - dy * sin_theta
        y_v = dx * sin_theta + dy * cos_theta

        return x_v, y_v

    def calc_lkahead(self, velocity):
        """
        Calculates the lookahead distance

        param velocity: The velocity of the vehicle
        """
        d_lookahead = self.k_lkahead * velocity + self.minimum_lookahead

        return d_lookahead

    def lkahead(self, path_xv, path_yv, d_lkahead, plot=False):
        """
        Searches the path for the point one lookahead distance away from the vehicle (if multiple points exist, choose the one further along the path)

        param path_xv: The x coordinates of the path in the vehicle local frame
        param path_yv: The y coordinates of the path in the vehicle local frame
        param d_lkahead: The lookahead distance
        param plot: a boolean of whether to plot the result

        return: The index of the target point
        """
        # Calculate the distance to the vehicle refrence point for each point on the path
        ysq = np.power(path_xv, 2)  
        xsq = np.power(path_yv, 2)
        distances = np.sqrt(xsq + ysq)

        # Calculate how close each point is to the lookahead distance
        differences = np.abs(distances - d_lkahead)

        # Make a boolean mask of whether all the points are ahead of the vehicle
        ahead_of_vehicle = path_yv < self.vehicle_length
        
        # Set all the points that are not ahead of the vehicle to a high value to preclude selection
        differences[ahead_of_vehicle] = 1000

        if False not in ahead_of_vehicle:
            logger.warning("PURE PURSUIT: Lookahead algorithm could not find any path points ahead of vehicle")

        # Get the index of the path point closest to the lookahead distance
        target_ind = np.argmin(differences)  # TODO: Determine how argmin decides between equally small values

        if plot:
            #figure, axis1 = plt.subplots(1,1)
            #axis1.imshow(self.virtual_img)
            #axis1.plot([self.vorigin_x, self.pic_pixx[self.target_ind]], [self.vorigin_y, self.pic_pixy[self.target_ind]], marker = 'o')
            #axis1.plot(self.pic_pixx, self.pic_pixy)
            #(self.pic_pixx[self.target_ind], self.pic_pixy[self.target_ind])
            #axis1.set_title("Lookahead Line")
            #plt.show()
            pass
        
        return target_ind

    def calc_circ_rad(self, path_xv, path_yv, t_ind, d_lkahead, plot=False):
        """
        Calculates the radius of the pure pursuit steering circle

        param path_xv: The x coordinates of the path in the vehicle local frame
        param path_yv: The y coordinates of the path in the vehicle local frame
        param t_ind: The index of the target point

        return: The radius of the pure pursuit steering circle
        """
        # Calculate the angle between the heading of the vehicle and the lookahead line
        alpha = np.arctan(path_xv[t_ind] / path_yv[t_ind])

        # Calculate the radius of the steering circle
        str_rad = d_lkahead / (2*np.sin(alpha))

        # Determine the direction (right or left) of the steering circle
        if (path_xv[t_ind] > 0):
            direction = True
        else:
            direction = False

        if plot:
            """
            if direction:
                corigin = self.vorigin_x + (str_rad // self.x_p2m)
            else:
                corigin = self.vorigin_x + (str_rad // self.x_p2m)

            figure, axis1 = plt.subplots(1,1)
            axis1.imshow(self.virtual_img)
            str_circle = Ellipse((corigin, self.vorigin_y), width=2 * (str_rad // self.x_p2m), height=2 * (str_rad // self.y_p2m), fill=False, color='red')
            #axis1.plot([self.vorigin_x, self.pic_pixx[self.target_ind]], [self.vorigin_y, self.pic_pixy[self.target_ind]], marker = 'o')
            #axis1.plot(self.pic_pixx, self.pic_pixy)
            axis1.add_artist(str_circle)
            axis1.set_title("Lookahead Line and Steering Circle")
            plt.show()
            """
            pass

        return str_rad, direction

    def calc_steering_angle(self, str_rad):
        """
        Calculates the pure pursuit steering angle

        param str_rad: The radius of the pure pursuit steering angle

        return: The pure pursuit steering angle (rad)
        """
        steering_angle = np.arctan((1 / str_rad)*self.vehicle_length)
        
        return steering_angle

def pure_pursuit_pipeline(state, pursuit):
    """
    Calculates the steering angle to maintain path tracking based on the vehicles state and lane information

    param pursuit: The pursuit object

    return: The steering angle (rad)
    """
    logger.info(f"PURE PURSUIT: Starting pure pursuit pipeline")
    velocity = state.velocity
    image_x = state.lcl_x
    image_y = state.lcl_y
    image_yaw = state.calc_lcl_heading()

    path_x = state.path_x
    path_y = state.path_y

    logger.debug(f"PURE PURSUIT: path x range ({np.min(path_x)}, {np.max(path_x)}), path y range ({np.min(path_y)}, {np.max(path_y)})")
    logger.debug(f"PURE PURSUIT: image state [x: {image_x}, y: {image_y}, image_yaw: {image_yaw}]")

    # Transform the coordinates of the path from the image frame to the vehicle frame
    path_xv, path_yv = pursuit.frame_transform(path_x, path_y, image_x, image_y, image_yaw)

    # Calculate the lookahead distance (how far on the path to look for the steering point)
    lookahead_distance = pursuit.calc_lkahead(velocity)

    # Calculate the index of the lookahead point
    lookahead_ind = pursuit.lkahead(path_xv, path_yv, lookahead_distance, plot=False)

    # Calculate the radius of the circle that meets the lookahead point
    circle_rad, direction = pursuit.calc_circ_rad(path_xv, path_yv, lookahead_ind, lookahead_distance, plot=False)

    # Calculate the steering angle
    steering_angle = pursuit.calc_steering_angle(circle_rad)

    if direction:
        directioned_steering_angle = steering_angle
    else:
        directioned_steering_angle = -steering_angle

    logger.info(f"PURE PURSUIT: Finished pure pursuit pipeline")

    return directioned_steering_angle






    
    









    

        
        
    

        

        
        








