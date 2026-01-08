import os
import sys
import cv2
import datetime
import logging

from core.constants import IMAGE_HEIGHT, IMAGE_WIDTH, FRAMERATE

logger =  logging.getLogger(__name__)

class PiCameraStream:
    def __init__(self, capture_width=IMAGE_WIDTH,
                capture_height=IMAGE_HEIGHT,
                display_width=IMAGE_WIDTH,
                display_height=IMAGE_HEIGHT,
                framerate=FRAMERATE,
                flip_method=2):
        
        self.gstreamer_pipeline_str = (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink drop=True"
        )

        self.camera_stream = None
        self.frame_count = 0

    def spin_up_camera(self):
        """
        Spins up the PiCam using the objects gstreamer config string
        """
        self.camera_stream = cv2.VideoCapture(self.gstreamer_pipeline_str, cv2.CAP_GSTREAMER)

        if self.camera_stream.isOpened():
            return
        else:
            logger.error("CAMERA STREAM ERROR: Unable to open camera")
            sys.exit(1)

    def get_frame(self):
        """
        Gets a frame and timestamp from the camera
        """
        capture_time = datetime.datetime.now()
        ret, frame = self.camera_stream.read()

        if ret:
            return frame, capture_time
        else:
            logger.error("CAMERA STREAM ERROR: Failed to read frame")

    def record_frame(self, frame_tuple, frame_store_path):
        """
        Saves a frame and records its associated timestamp in a file

        frame_tuple: The image and its timestamp
        frame_store_path: The path to the directory where the frame should be stored
        """
        frame = frame_tuple[0]
        timestamp = frame_tuple[1]

        frame_name = f"frame_{self.frame_count}.png"
        frame_path = os.path.join(frame_store_path, frame_name)

        timestamp_path = os.path.join(frame_store_path, "timestamps.txt")

        cv2.imwrite(frame_path, frame)

        with open(timestamp_path, "a") as timestampstore:
            timestamp_string = f"{datetime.datetime.timestamp(timestamp)},{frame_name}"
            timestampstore.write(timestamp_string)
            timestampstore.write("\n")

        self.frame_count += 1

    def display_camera(self):
        """
        Displays the output of the PiCam
        """
        window_title = "PiCam Display"
        try:
            cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:

                ret, frame = self.camera_stream.read()
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            self.camera_stream.release()
            cv2.destroyAllWindows()