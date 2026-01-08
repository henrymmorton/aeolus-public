import argparse
import os
import struct
import cv2
import numpy as np

from core.constants import STATE_PACKER_FORMAT, CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, PPM, BEV_FOV_WIDTH, BEV_FOV_HEIGHT, CAMERA_GRID_Y_OFFSET, V_LENGTH
from core.cv.nueral_net.vision_pipeline.perspective_transform_measured import CameraPerspectiveTransform

POINT_RAD = 20
POINT_COLOR = (255, 0, 0)


# Initialize the perspective transformer
perspective_transform = CameraPerspectiveTransform(CAMERA_CALIBRATION_MEASUREMENTS, CAMERA_CALIBRATION_PIXELS, PPM, BEV_FOV_WIDTH, BEV_FOV_HEIGHT, CAMERA_GRID_Y_OFFSET)

# Calculate an expansion thats one half vehicle length behind the origin
vehicle_buffer_meters = np.array([[0, -V_LENGTH / 2]])
vehicle_buffer_pix = perspective_transform.vehicle2image_transform(vehicle_buffer_meters)
vehicle_buffer_BEV = perspective_transform.image2BEVimage_transform(vehicle_buffer_pix)
vehicle_buffer_BEV_y = vehicle_buffer_BEV[0][1]

class StateLog:
    def __init__(self, ts_nanoseconds, image_ts_nanoseconds, lcl_x, lcl_y, lcl_theta,
                 glb_x, glb_y, glb_heading,dist, velocity, steer_angle, accel_x, accel_y, ultra_distance):
        """
        Docstring for __init__
        
        :param self: Description
        :param ts_nanoseconds: Description
        :param image_ts_nanoseconds: Description
        :param lcl_x: Description
        :param lcl_y: Description
        :param lcl_theta: Description
        :param glb_x: Description
        :param glb_y: Description
        :param glb_heading: Description
        :param dist: Description
        :param velocity: Description
        :param steer_angle: Description
        :param accel_x: Description
        :param accel_y: Description
        :param ultra_distance: Description
        """
        
        # State variables
        self.ts_nanoseconds = ts_nanoseconds
        self.image_ts_nanoseconds = image_ts_nanoseconds
        self.lcl_x = lcl_x
        self.lcl_y = lcl_y
        self.lcl_theta = lcl_theta
        self.glb_x =  glb_x
        self.glb_y = glb_y
        self.glb_heading = glb_heading
        self.dist = dist
        self.velocity = velocity
        self.steer_angle = steer_angle
        self.accel_x = accel_x
        self.accel_y = accel_y
        self.ultra_distance = ultra_distance
        
        # Image variables
        self.image_path = None
        self.image_name = None

    def transform_pose_to_BEV(self, perspective_transform):
        """
        Docstring for transform_pose_to_BEV
        
        :param self: Description
        :param perspective_transform: Description
        """
        meters_coords = np.array([[self.lcl_x, self.lcl_y]])
        pix_coords = perspective_transform.vehicle2image_transform(meters_coords)
        BEV_pix_coords = perspective_transform.image2BEVimage_transform(pix_coords)
        lcl_x_BEV_pixels = BEV_pix_coords[0][0]
        lcl_y_BEV_pixels = BEV_pix_coords[0][1]
        
        return np.array([lcl_x_BEV_pixels, lcl_y_BEV_pixels])

def load_image(image_path, perspective_transform):
    """
    Loads an image from a file, then transforms it to a birds eye view
    
    :param image_path: Path to the image
    :param perspective_transform: The BEV perspective transformer
    """
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Unable to load image at: {image_path}")
    image_BEV = perspective_transform.warp_to_BEV(image)

    return image_BEV

def pad_image(image, points):

    height, width = image.shape[:2]

    x_coords = points[:, 0]
    y_coords = points[:, 1]

    min_x = int(x_coords.min())
    min_y = int(y_coords.min())
    max_x = int(x_coords.max())

    # Calculate the maximum between the furthest down point and the vehicle buffer
    # (black space that represents the space that the vehicle occupies behind the camera)
    max_y = max(int(y_coords.max()), int(vehicle_buffer_BEV_y))

    # Calculate the padding values
    pad_left = max(0, -min_x)
    pad_top = max(0, -min_y)
    pad_right = max(0, (max_x - (width - 1)))
    pad_bottom = max(0, (max_y - (height - 1)))

    padded_points = points.copy()
    padded_points[:, 0] += pad_left
    padded_points[:, 1] += pad_top

    padded_image = cv2.copyMakeBorder(image, top=pad_top, bottom=pad_bottom, left=pad_left, right=pad_right, borderType=cv2.BORDER_CONSTANT, value=(0,0,0))

    return padded_image, padded_points
    
def load_image_log(image_log_path, image_directory):
    """
    Docstring for load_image_log
    
    :param image_log_path: Description
    :param image_directory: Description
    """

    image_paths = {}

    with open(image_log_path) as f:
        for line in f:
            timestamp_str, image_name = line.rstrip("\n").split(",", 1)
            timestamp = int(timestamp_str)
            image_path = os.path.join(image_directory, image_name)
            image_paths[timestamp] = image_path

    return image_paths

def load_state_logs(state_log_path, image_paths):
    """
    Docstring for load_state_logs
    
    :param state_log_path: Description
    :param image_paths: Description
    """

    state_log_size = struct.calcsize(STATE_PACKER_FORMAT)
    state_logs = []

    with open(state_log_path, "rb") as f:
        while True:
            packed_log = f.read(state_log_size)
            if not packed_log:
                break
            if len(packed_log) != state_log_size:
                raise ValueError("Corrupt or partial state log")
            
            log = struct.unpack(STATE_PACKER_FORMAT, packed_log)
            state_log = StateLog(log[0], log[1], log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9], log[10], log[11], log[12], log[13])

            if state_log.image_ts_nanoseconds in image_paths:
                state_log.image_path = image_paths[state_log.image_ts_nanoseconds]
            else:
                raise ValueError(f"No matching image found for state with image timestamp: {state_log.image_ts_nanoseconds}")

            state_logs.append(state_log)

    return state_logs

def draw_point(image, x, y):
    """
    Docstring for draw_point
    
    :param image: Description
    :param x: Description
    :param y: Description
    """
    center = (int(y), int(x))
    print(f"plotting point at: {center}")
    cv2.circle(image, center, radius = POINT_RAD, color = POINT_COLOR, thickness = -1)

def draw_points(image, points):
    """
    Docstring for draw_points
    
    :param image: Description
    :param points: Description
    """
    for x, y in points:
        center = (int(x), int(y))
        
        cv2.circle(image, center, radius = POINT_RAD, color = POINT_COLOR, thickness = -1)

def plot_states(state_log_path, image_log_path, image_directory, save_directory):
    """
    Docstring for plot_states
    
    :param state_log_path: Description
    :param image_log_path: Description
    :param image_directory: Description
    :param save_directory: Description
    """

    # Construct the save path for the plotted images and create the directory
    image_log_name = os.path.basename(image_log_path)
    date_string = image_log_name.split("_")[-1].split(".")[0]
    image_save_dir_name = f"plotted_images_{date_string}"
    image_save_dir = os.path.join(save_directory, image_save_dir_name)
    os.makedirs(image_save_dir, exist_ok=True)

    # Load the image logs to get the timestamp-image association and then load the state logs and associate their images
    image_paths = load_image_log(image_log_path, image_directory)
    state_logs = load_state_logs(state_log_path, image_paths)

    # Fetch the first image
    current_image_path = state_logs[0].image_path
    current_image_BEV = load_image(current_image_path, perspective_transform)
    current_image_poses = []

    for log in state_logs:
        # Case where we get a new image
        if log.image_path != current_image_path:
            # Convert the list of points to numpy array and then wipe the list
            poses_array = np.vstack(current_image_poses)
            current_image_poses = []

            # Pad the image to allow all points to appear
            padded_image, padded_poses = pad_image(current_image_BEV, poses_array)
            # Plot the points
            draw_points(padded_image, padded_poses)
        
            # Save the old image
            image_name = os.path.basename(current_image_path)
            save_path = os.path.join(image_save_dir, image_name)
            cv2.imwrite(save_path, padded_image)

            # Load the new image
            current_image_path = log.image_path
            current_image_BEV = load_image(current_image_path, perspective_transform)
        
        current_image_poses.append(log.transform_pose_to_BEV(perspective_transform))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="The main for the aeolus autonomous vehicle")
    parser.add_argument('-i', '--image_logs')
    parser.add_argument('-state', '--state_logs')
    parser.add_argument('-id', '--image_directory')
    parser.add_argument('-s', '--save_dir')
    args = parser.parse_args()

    plot_states(args.state_logs, args.image_logs, args.image_directory, args.save_dir)


