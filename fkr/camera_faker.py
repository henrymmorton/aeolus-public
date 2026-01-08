
import numpy as np
import cv2 as cv
from datetime import datetime 
import time
import os
from core.constants import LOG_DIR

class StreamFaker:
    
    def __init__(self, stream_path, sim_now):
        """
        Camera stream faker constructor

        param stream_path: The path to the image recording location
        param sim_now: Whether to overwrite frame capture timestamps wiht current time
        """
        self.frame_num = 0
        self.stream_path = stream_path

        timestamps_path = os.path.join(stream_path, "timestamps.txt")
        with open(timestamps_path, 'r') as timestamps_file:
            self.timestamps = timestamps_file.readlines()

        self.sim_now = sim_now
        self.image_store = None

    def start_restamping(self):
        # Open the state store file
        image_store_name = f"image_logs_{datetime.now().strftime('%H-%M-%S')}.txt"
        image_store_path = os.path.join(LOG_DIR, image_store_name)
        self.image_store = open(image_store_path, "a")

    def get_video_frame(self):
        """
        Gets a frame from the video then waits 1/30th of a second to simulate real frame rate
        """
        timestamp_and_frame_num = self.timestamps[self.frame_num].split(',')

        # If we are doing a current sim, overwrite the capture timestamp with now
        if self.sim_now:
            capture_time = datetime.now()
            capture_time_ns = int(capture_time.timestamp() * 1e9)
            timestamp_string = f"{capture_time_ns},{timestamp_and_frame_num[1].strip()}"
            self.image_store.write(timestamp_string)
            self.image_store.write("\n")
            self.image_store.flush()
        else:
            capture_time = datetime.fromtimestamp(float(timestamp_and_frame_num[0]))

        frame_num = timestamp_and_frame_num[1].strip()
        frame_path = os.path.join(self.stream_path, frame_num)
        frame = cv.imread(frame_path)

        self.frame_num += 1
        time.sleep(1/30)
        return frame, capture_time


    
    