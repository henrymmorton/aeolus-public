import cv2
import numpy as np

class CameraPerspectiveTransform:

    def __init__(self, measurements, pixels, pixels_per_meter, fov_width, fov_height, camera_image_y_offset):
        self.ppm = pixels_per_meter
        self.fov_width = fov_width
        self.fov_height = fov_height
        self.camera_image_y_offset = camera_image_y_offset

        ransac_const = 5.0 # TODO: Figure out what this does

        # Calculate the homography matrix between the camera view and the vehicle local coordinate system
        self.H, _ = cv2.findHomography(pixels, measurements, cv2.RANSAC, ransac_const)
        self.H_inv = np.linalg.inv(self.H)

        # Calculate the homography matrix between the camera view and a birds eye, pixel based view
        picture_origin_shift = np.array([-self.fov_width/2, self.fov_height + self.camera_image_y_offset])
        picture_frame = measurements - picture_origin_shift # Shift the origin from the bottom middle to the top left most point (typical picture origin)
        picture_frame[:,1] = - picture_frame[:,1] # Flip the y-axis to be positive pointing down
        picture_scaled = picture_frame * pixels_per_meter
        
        self.H_pic, _ = cv2.findHomography(pixels, picture_scaled, cv2.RANSAC, ransac_const)

    def warp_to_BEV(self, image):
        """
        Transforms the image from the perspective of the camera to a birds eye view and compensates for the cameras warp.
        This method should yield an image with a constant pixel to meter ratio

        param image: The camera perspective image
        returns: The birdseye view image
        """
        fov_dims = (int(self.ppm * self.fov_width), int(self.ppm * self.fov_height))
        birdseye_view = cv2.warpPerspective(image, self.H_pic, fov_dims)

        return birdseye_view
    
    @staticmethod
    def general_perspective_transform(coords, H):
        """
        Transforms coordinates from one frame to another

        param coords: A numpy array of N sets of coordinates of shape (N,2)
        param H: The homogrpahy matrix representing the transform from one frame to another
        returns: A numpy array of coordinates in the new frame
        """
        # Reshape the coords to be compatible with opencv's perspective transform
        shaped_coords = np.asarray(coords, dtype=np.float32).reshape(-1, 1, 2)

        # Transform the coordinates to the vehicle local frame
        transformed_coords = cv2.perspectiveTransform(shaped_coords, H)

        # Return the coords to their normal shape
        transformed_coords = transformed_coords.reshape((-1, 2))

        return transformed_coords
    
    def image2vehicle_transform(self, pixel_coords):
        """
        Transforms pixel coordinates from the image frame into the vehicle local frame
        This method should yield coordinates in meters

        param pixel_coords: A numpy array of N sets of pixel coordinates of shape (N,2)
        returns: A numpy array of coordinates in the vehicle local frame
        """

        return self.general_perspective_transform(pixel_coords, self.H)
    
    def vehicle2image_transform(self, meters_coords):
        """
        Transforms pixel coordinates from the vehicle local frame into the image frame
        This method should yield coordinates in pixels

        param meters_coords: A numpy array of N sets of meters coordinates of shape (N,2)
        returns: A numpy array of pixel coordinates
        """
        pixel_coords = self.general_perspective_transform(meters_coords, self.H_inv)

        # Round since pixel coords are integers
        pixel_coords = pixel_coords.round()

        return pixel_coords
    
    def image2BEVimage_transform(self, pixel_coords):
        """
        Transforms pixel coordinates from the image frame to the image BEV frame
        This method should yield coordinates in pixels

        param pixel_coords: A numpy array of N sets of pixel coordinates of shape (N,2)
        returns: A numpy array of pixel coordinates
        """
        bev_pixel_coords = self.general_perspective_transform(pixel_coords, self.H_pic)

        # Round since pixel coords are integers
        bev_pixel_coords = bev_pixel_coords.round()

        return bev_pixel_coords






    
