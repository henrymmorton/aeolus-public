#from core.cv.nueral_net.vision_pipeline.perspective_transform_derived import CameraPerspectiveTransform


class Vehicle:

    def __init__(self, length, width, wheel_radius, axel2axel_length, camera_height, cam_ax_offset, cam2im_plane, intrinsic_camera_matrix):

        self.length = length
        self.width = width
        self.wheel_rad = wheel_radius
        self.a2a_length = axel2axel_length
        self.cam_height = camera_height
        self.ca_offset = cam_ax_offset
        self.cam2im = cam2im_plane