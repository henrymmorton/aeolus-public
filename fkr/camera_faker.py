
import numpy as np
import cv2 as cv
import datetime 
import time
import os

class StreamFaker:
    
    def __init__(self, stream_path):
        """
        Camera stream faker constructor

        param stream_path: The path to the image recording location
        """
        self.frame_num = 0
        self.stream_path = stream_path

        timestamps_path = os.path.join(stream_path, "timestamps.txt")
        with open(timestamps_path, 'r') as timestamps_file:
            self.timestamps = timestamps_file.readlines()

    def get_video_frame(self):
        """
        Gets a frame from the video then waits 1/30th of a second to simulate real frame rate
        """
        timestamp_and_frame_num = self.timestamps[self.frame_num].split(',')

        capture_time = datetime.datetime.fromtimestamp(float(timestamp_and_frame_num[0]))
        frame_num = timestamp_and_frame_num[1].strip()

        frame_path = os.path.join(self.stream_path, frame_num)
        frame = cv.imread(frame_path)

        self.frame_num += 1
        time.sleep(1/30)
        return frame, capture_time


    
    