import cv2
import np
from perspective_transform_measured import CameraPerspectiveTransform


image_source = "/home/henry/git/aeolus_dev/core/cv/camera_calibration/calibration_images/calibration_image_2025-07-08_18-25-33.png"
image = cv2.imread(image_source)

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

my_transform = CameraPerspectiveTransform(CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, PPM)

width = 0.443 # meters
height = 0.664 # meters
dims = (int(PPM*width), int(PPM*height))

picture_BEV = my_transform.warp_to_BEV(image, dims[0], dims[1])

cv2.imwrite("/home/henry/git/aeolus_dev/core/cv/camera_calibration/calibration_images/image_BEV_UNIT.png", picture_BEV)

test_pixels = np.array([[393, 380],
                        [798, 407],
                        [360, 209],
                        [806, 257]])

test_coords = my_transform.perspective_transform(test_pixels)