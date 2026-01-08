import cv2
import numpy as np

class CameraPerspectiveTransform:

    def __init__(self, intrinsic_camera_matrix, camera_height):

        # Calculate the homography matrix to perform the perspective transform on the image
        K = intrinsic_camera_matrix

        h = 0.15

        # Pitch angle
        theta = np.radians(20)

        # Pitch matrix
        Rx = np.array([[1,0,0],
                       [0, np.cos(theta), -np.sin(theta)],
                       [0, np.sin(theta), np.cos(theta)]])

        # The camera will always be pointing forwards so the rotation matrix is fixed
        R_level = np.array([[1, 0, 0],
                     [0, 0, -1],
                     [0, 1, 0]])
        
        R = R_level @ Rx

        r1 = R[:,0]
        r2 = R[:,1]
        r3 = R[:,2]
        t = -h * r3

        T = np.array([[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]])
        
        RT = np.column_stack((r1, r2, t))

        self.H = K @ RT

        # self.H = self. H @ T

        self.H_inv = np.linalg.inv(self.H)

   
    def perspective_transform(self, image, output_width, output_height):
        """
        Transforms the image from the perspective of the camera to a birds eye view and compensates for the cameras warp.
        This method should yield an image with a constant pixel to meter ratio

        param image: The camera perspective image
        param output_width: The desired width of the transformed image
        param output_height: The desired height of the transformed_image
        returns: The birdseye view image
        """

        birdseye_view = cv2.warpPerspective(image, self.H_inv, (output_height, output_width))

        return birdseye_view
    
