import os
import sys
import time
import numpy as np
from numpy.polynomial import Polynomial
from scipy.optimize import root_scalar
import matplotlib.pyplot as plt
import cv2
from PIL import Image as im

from core.constants import FIT_DEG, MIN_LINE_LENGTH, MORPH_KERNAL_SIZE, LOG_DIR, MAX_PATH_VALIDITY
import logging

logger =  logging.getLogger(__name__)

hard_image_path = "/Users/henrymorton/Downloads/image0045.png"
hard_label_path = "/Users/henrymorton/Downloads/prediction0045.npy"

def invert_polynomial(p, y, guess):
    """
    """
    fp = p.deriv()
    f = lambda x: p(x) - y
    sol = root_scalar(f, x0=guess, fprime=fp, method="newton")
    if not sol.converged:
        logger.error(f"Root finding failed for y={y} with fit {p} and guess {guess}")
        return None
    else:
        return sol.root

def invert_polynomial_array(p, y_coords, range):
    """
    Calculate the x coordinates of a polynomial given np array of y coordinates and a x domain
    Uses newton method with the mean of the xdomain as the first guess

    param p: A polynomial
    param y_coords: A numpy array of y coordinates for which you want to solve for x
    param range: The x domain of the polynomial 
    """

    x_coords = np.zeros(y_coords.shape)
    
    fp = p.deriv()
    guess = np.mean(range)

    for i, y in enumerate(y_coords):
        f = lambda x: p(x) - y
        sol = root_scalar(f, x0=guess, fprime=fp, method="newton")
        if not sol.converged:
            raise RuntimeError(f"Root finding failed for y={y}")
        x_coords[i] = sol.root
        guess = sol.root

    return x_coords

class Lane:
    """
    Digital model of a track lane
    """
    
    def __init__(self, timestamp):
        """
        Lane constructor

        param timestamp: The timestamp when the image was taken
        """
        # Timestamp of the moment in time the lane represents
        self.timestamp = timestamp

        # Holds booleans which flag if the left and right lanes have been detected
        self.lane_detected = [False, False, False, False]

        # An identifier for which path was selected: 0=center of lane, 1=left of camera, 2=right of camera       
        self.p_iden = None

        # The lane fits produced by the lane detection pipeline
        self.center_fits = None

        # The center path coordinates
        self.path_x = None
        self.path_y = None

    @staticmethod
    def gen_poly_points_range(fit, x_range, numpoints = 500):
        """
        Generates the xy coordinates of a polynomial fit within a min max range

        param fit: A numpy polynomial
        param x_range: The range of x values to generate over
        param numpoints: The number of points to generate (controls granularity)

        return: Arrays of ydim of y coordinates and x coordinates for the fit
        """
        min_x = x_range[0]
        max_x = x_range[1]

        x_coords = np.linspace(min_x, max_x, numpoints)
        y_coords = fit(x_coords)

        return x_coords, y_coords
    
    @staticmethod
    def gen_poly_points_y_range(fit, x_domain, y_range, numpoints = 500):
        """
        Generates the xy coordinates of a polynomial fit within a min max y value range
        Functions by root finding for x for the y_range endpoints then evaluating the polynomial between those

        param fit: A numpy polynomial
        param x_domain: The valid range of x values
        param y_domain: The desired range of y values
        param numpoints: The number of points you want within the range (granularity)

        return: Arrays of x_coords and y_coords
        """
        guess = np.mean(x_domain)

        x_bound_0 = invert_polynomial(fit, y_range[0], guess)
        x_bound_1 = invert_polynomial(fit, y_range[1], guess)

        if x_bound_0 is None or x_bound_1 is None:
            return None, None

        x_coords = np.linspace(x_bound_0, x_bound_1, numpoints)
        y_coords = fit(x_coords)
        
        return x_coords, y_coords
    
    @staticmethod
    def gen_poly_points_y_range_slow(fit, x_domain, y_range, numpoints = 500):
        """
        Generates the xy coordinates of a polynomial fit within a min max y value range
        Functions by root finding x for each y

        param fit: A numpy polynomial
        param x_domain: The valid range of x values
        param y_domain: The desired range of y values
        param numpoints: The number of points you want within the range (granularity)

        return: Arrays of x_coords and y_coords
        """
        y_coords = np.linspace(y_range[0], y_range[1], numpoints)
        x_coords = invert_polynomial_array(fit, y_coords, x_domain)
        
        return x_coords, y_coords

    def post_process_inferences(self, lane_inferences):
        """
        Post-processes lane inference to filter out noise and fill any small gaps

        param lane_inferences: A numpy array containing the lane information for all lanes

        return: A post processed version of the lane inference (noise removed and gaps filled)
        """
        # Cast as smaller datatype to make compatible
        lane_inferences = lane_inferences.astype('uint8')

        # Trim out the 0 (not a lane) class
        lane_inferences = lane_inferences[1:5]

        # Calculate the contours for each of the lane lines
        for lane_index in range(0, lane_inferences.shape[0]):
            lane_contours, _ = cv2.findContours(lane_inferences[lane_index,:,:], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Iterate through each contour
            for contour in lane_contours:
                # Get the bounding box of the contour
                _, _, w, h = cv2.boundingRect(contour)

                # TODO: Explore possibility of removing noise by using this to redraw mask

                # If the width or height of the lane bounding box exceeds the threshold, mark the lane as detected
                if w > MIN_LINE_LENGTH or h > MIN_LINE_LENGTH:
                    self.lane_detected[lane_index] = True

        # Set up the kernal for morphological closing
        kernel = np.ones((MORPH_KERNAL_SIZE, MORPH_KERNAL_SIZE), np.uint8)

        # Loop through and replace lanes in place with a filtered version
        for lane_num in range(lane_inferences.shape[0]):
            lane_inferences[lane_num,:,:] = cv2.morphologyEx(lane_inferences[lane_num,:,:], cv2.MORPH_CLOSE, kernel)

        return lane_inferences
    
    def perspective_transform(self, img, perspective_transformer, plot=False):
        """
        Transform the image to a birds eye view

        param img: the input image
        param perspecitve_transformer (CameraPerspectiveTransform): Transformer to move between image, birds eye image, and vehicle local frames
        param plot: a boolean of whether or not plot the result

        return: A birds eye view of the region of interest
        """
        pwarped = perspective_transformer.warp_to_BEV(img)
        
        # Display the perspective warped image
        if plot:
            figure, (axis1, axis2) = plt.subplots(2,1)
            img_display = img.copy()
            roi_display = pwarped.copy()
            figure.set_size_inches(10, 6)
            figure.tight_layout(pad=3.0)
            axis1.imshow(img, cmap='gray')
            axis1.set_title("Original Perspective")
            axis2.imshow(pwarped, cmap='gray')
            axis2.set_title("Transformed Perspective")
            plt.show()

        return pwarped
    
    def transform_inferences(self, inference, perspective_transformer):
        """
        Transform all lane inferences from an image to a pixel coordinate set, and then from pixel coords to vehicle local coords

        param lane_inference: the input lane inferences
        param perspecitve_transformer (CameraPerspectiveTransform): Transformer to move between image, birds eye image, and vehicle local frames

        return: A birds eye view of the region of interest
        """
        lane_line_coords_list = [False, False, False, False]

        for lane_index in range(inference.shape[0]):
            if self.lane_detected[lane_index]:
                pixel_coords = np.argwhere(inference[lane_index,:,:])
                pixel_coords[:, [0, 1]] = pixel_coords[:, [1, 0]]
                coords_vehicle_local = perspective_transformer.image2vehicle_transform(pixel_coords)
                lane_line_coords_list[lane_index ] = coords_vehicle_local

        return lane_line_coords_list
    
    def calc_weights(self, coords, y_max):
        """
        Calculates the weight of a coordinate for polynomial fitting based on its coordinates

        param coords: The points xy coords
        param y_max: The maximum y value in the image

        return: The coordinates weight
        """
        weights = y_max - coords[:,0] + 100

        return weights
    
    def fit_lane_line(self, lane_coords):
        """
        Generates a polynomial fit for a lane line using the inferred lane coordinates
        The fit generates the x points as a function of y because the lane lines will often be nearly verticle

        param lane_coords: A numpy array containing the inferred lane line coordinates

        return: The lane fit and the y domain over which it is applicable
        """
        # Get the min and max x and y values of the lanes
        min_max_x = (np.min(lane_coords[:, 0]), np.max(lane_coords[:, 0]))
        min_max_y = (np.min(lane_coords[:, 1]), np.max(lane_coords[:, 1]))

        # Calculate a per-pixel weight array
        # TODO: Determine if we want to keep weights
        # weights = self.calc_weights(lane_coords, min_max_y[1])

        # Fit a polynomial to each set of lane coordinates (x as a function of y)
        lane_fit = Polynomial.fit(lane_coords[:, 1], lane_coords[:, 0], FIT_DEG)

        return lane_fit, min_max_x
    
    def fit_lanes(self, lane_coords):
        """
        Generates a polynomial fit for the left and right lane lines and the center of the lane
        Functions by fitting predictions for the left and right lines ->
        Generating points for those predictions -> 
        Averaging the points to get center points ->
        Fitting the center points to get a lane center fit

        param lane_coords: A list of numpy arrays containing the inferred coordinate of the lane line pixels
        """
        logger.debug("LANE DETECTION STATUS: Started fitting lanes")

        lane_fit_list = []
        center_fit_list = []
        common_y_domain_list = []

        # Fit all the lane lines
        for index, lane in enumerate(lane_coords):
            if self.lane_detected[index]:
                lane_fit, lane_domain = self.fit_lane_line(lane)
                lane_fit_list.append(lane_fit)
            else:
                lane_fit_list.append(False)

        # Fit the center lines by taking the mean of their left and right pairs
        for lane_index in range(len(lane_fit_list) - 1):
            if lane_fit_list[lane_index] and lane_fit_list[lane_index + 1]:
                common_y_domain = [max(lane_fit_list[lane_index].domain[0], lane_fit_list[lane_index + 1].domain[0]), min(lane_fit_list[lane_index].domain[1], lane_fit_list[lane_index + 1].domain[1])]
                # expansive_common_y_domain = (min(left_fit.domain[0], right_fit.domain[0]), max(left_fit.domain[1], right_fit.domain[1]))
                
                l_y, l_x = self.gen_poly_points_range(lane_fit_list[lane_index], common_y_domain)
                r_y, r_x = self.gen_poly_points_range(lane_fit_list[lane_index + 1], common_y_domain)

                # Average to get the center coordinates
                c_x = (l_x + r_x) / 2
                c_y = (l_y + r_y) / 2

                # Again fit x as a function of y (due to near vertical lines)
                center_fit = Polynomial.fit(c_y, c_x, FIT_DEG)
                logger.debug("LANE DETECTION STATUS: Finished fitting lanes")
                center_fit_list.append(center_fit)

                # Set the common y domains to 0 -> the lesser of MAX_PATH_VALIDITY or the fit domain
                if (common_y_domain[1] < MAX_PATH_VALIDITY):
                    common_y_domain = (1, common_y_domain[1])
                else:
                    common_y_domain = (1, MAX_PATH_VALIDITY)

                common_y_domain_list.append(common_y_domain)
            else:
                center_fit_list.append(False)
                common_y_domain_list.append(False)

        return lane_fit_list, center_fit_list, common_y_domain_list
    
    def plot_inferences(self, inferences, save = False, save_name = None):
        """
        Plots a set of left and right inferences

        param left_inference: The set of coordinates [[x_coords],[y_coords]] for the left inference
        param right_inference: The set of coordinates [[x_coords],[y_coords]] for the right inference
        param save: Whether to save the plot
        param save_name: Where to save the plot
        """
        fig, ax = plt.subplots()

        for lane_index, lane_inference in enumerate(inferences):
            if self.lane_detected[lane_index]:
                scaled_inference = lane_inference * 255
                ax.scatter(scaled_inference[:, 0], scaled_inference[:, 1])
                ax.set_aspect('equal')
                ax.set_title("Raw Inferences")

        if save:
            plt.savefig(os.path.join(LOG_DIR, save_name))
        else:    
            plt.show()
    
    def plot_fit(self, image, center_fits, line_fits, common_domains, perspective_transformer):
        """
        Plots a set of left, right and center fits on an image

        param image: The image to plot on
        param fits: A tuple of the center, left, and right fits
        param perspecitve_transformer (CameraPerspectiveTransform): Transformer to move between image, birds eye image, and vehicle local frames
        """
        figure1, (axis1) = plt.subplots()
        figure1.set_size_inches(10, 10)
        figure1.tight_layout(pad=3.0)
        axis1.set_aspect('equal')
        axis1.set_title("Vehicle Local Lane Lines and Center Path (Meters)")

        center_coords = []
        line_coords = []

        for center_index, center_line_fit in enumerate(center_fits):
            if center_line_fit:
                plot_y, plot_x = self.gen_poly_points_range(center_line_fit, common_domains[center_index])
                c_path = np.column_stack((plot_x, plot_y))
                center_coords.append(c_path)
                axis1.plot(c_path[:, 0], c_path[:, 1], color='blue', linewidth = 5)
            else:
                center_coords.append(False)
        
        for line_fit in line_fits:
            if line_fit:
                line_plot_y, line_plot_x = self.gen_poly_points_range(line_fit, line_fit.domain)
                line_path = np.column_stack((line_plot_x, line_plot_y))
                line_coords.append(line_path)
                axis1.plot(line_path[:, 0], line_path[:, 1], color='red', linewidth = 3)
            else:
                line_coords.append(False)

        plt.show()

        figure2, (axis2) = plt.subplots(1,1)
        figure2.set_size_inches(10, 10)
        figure2.tight_layout(pad=3.0)
        axis2.set_aspect('equal')
        axis2.set_title("Pixel Lane Lines and Center Path Pixel Frame")

        pix_center_coords = []
        pix_line_coords = []

        for center_coords_i in center_coords:
            if center_coords_i is not False:
                center_coords_i_pix = perspective_transformer.vehicle2image_transform(center_coords_i)
                pix_center_coords.append(center_coords_i_pix)
                axis2.plot(center_coords_i_pix[:, 0], center_coords_i_pix[:, 1], color='blue', linewidth = 5)
            else:
                pix_center_coords.append(False)

        for line_coords_i in line_coords:
            if line_coords_i is not False:     
                line_coords_i_pix = perspective_transformer.vehicle2image_transform(line_coords_i)
                pix_line_coords.append(line_coords_i_pix)
                axis2.plot(line_coords_i_pix[:, 0], line_coords_i_pix[:, 1], color='red', linewidth = 3)
            else:
                pix_line_coords.append(False)

        plt.show()

        figure3, (axis3) = plt.subplots(1,1)
        figure3.set_size_inches(10, 10)
        figure3.tight_layout(pad=3.0)
        axis3.set_aspect('equal')
        axis3.set_title("Pixel Lane Lines and Center Path Overlay")
        axis3.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        BEV_pix_center_coords = []
        BEV_pix_line_coords = []

        for pix_center_coords_i in pix_center_coords:
            if pix_center_coords_i is not False:
                center_coords_i_BEV_pix = perspective_transformer.image2BEVimage_transform(pix_center_coords_i)
                BEV_pix_center_coords.append(center_coords_i_BEV_pix)
                axis3.plot(center_coords_i_BEV_pix[:, 0], center_coords_i_BEV_pix[:, 1], color='blue', linewidth = 5)
            else:
                BEV_pix_center_coords.append(False)

        for pix_line_coords_i in pix_line_coords:
            if pix_line_coords_i is not False:
                line_coords_i_BEV_pix = perspective_transformer.image2BEVimage_transform(pix_line_coords_i)
                BEV_pix_line_coords.append(line_coords_i_BEV_pix)
                axis3.plot(line_coords_i_BEV_pix[:, 0], line_coords_i_BEV_pix[:, 1], color='red', linewidth = 3)
            else:
                BEV_pix_line_coords.append(False)

        plt.show()

def lane_detection_pipeline(persepctive_transformer, inference, image_time, image = None, plot = False):
    """
    Takes in an image and its neural net lane inference data and produces a lane object

    param perspecitve_transformer (CameraPerspectiveTransform): Transformer to move between image, birds eye image, and vehicle local frames
    param inference: The neural net lane inference
    param image_time: The timestamp associated with the image
    param image: The original image
    param plot: Whether to plot the result
    """
    logger.info("LANE DETECTION STATUS: Starting lane detection pipeline")
    current_lane = Lane(image_time)

    t_mark = time.time()

    # Post process the inference image to highlight lanes
    inference = current_lane.post_process_inferences(inference)
    missing_left_or_right = (not current_lane.lane_detected[1]) or (not current_lane.lane_detected[2])

    # If we don't detect a right and left lane, junk the lane object and return None
    if missing_left_or_right:
        logger.warning("LANE DETECTION: Left or right lane not detected. Returning none for current images lane object")
        return None
    else:
        logger.warning(f"LANE DETECTION: Successfully detected left and right lane {round(time.time() - t_mark, 5)}s")
        t_mark = time.time()

    # Transform the inference from a camera perspective  boolean image to a set of vehicle local coordinates for lane points
    vehicle_local_inferences = current_lane.transform_inferences(inference, persepctive_transformer)

    logger.debug(f"LANE DETECTION: Transformed inferences {round(time.time() - t_mark, 5)}s")
    t_mark = time.time()

    if plot:
        current_lane.plot_inferences(vehicle_local_inferences)

    # Fit all of the lanes
    line_fits, center_fits, common_y_domains = current_lane.fit_lanes(vehicle_local_inferences)
    current_lane.center_fits = center_fits

    logger.debug(f"LANE DETECTION: Fit lanes {round(time.time() - t_mark, 5)}s")
    t_mark = time.time()

    # Generate the points for the center path fit
    path_y, path_x = current_lane.gen_poly_points_range(current_lane.center_fits[1], common_y_domains[1], numpoints = 1000)

    logger.debug(f"LANE DETECTION: Generated polypoints {round(time.time() - t_mark, 5)}s")

    # Set the path coordinate variables
    current_lane.path_x = path_x
    current_lane.path_y = path_y

    if plot:
        # Transform the original image to a birds eye view and store it in the lane object
        birds_eye = current_lane.perspective_transform(image, persepctive_transformer)
        current_lane.plot_fit(birds_eye, center_fits, line_fits, common_y_domains, persepctive_transformer)


    logger.info("LANE DETECTION STATUS: Finished lane detection pipeline")

    return current_lane