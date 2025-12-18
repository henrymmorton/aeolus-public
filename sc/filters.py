from collections import deque
from statistics import median, mean
import math


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.average_queue = deque()

    def reset(self):
        self.average_queue.clear()

    def update(self, x):
        if len(self.average_queue) < self.window_size:
            self.average_queue.append(x)
        else:
            self.average_queue.pop()
            self.average_queue.append(x)

        return mean(self.average_queue)

class MedianFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.median_queue = deque()

    def reset(self):
        self.median_queue.clear()

    def update(self, x):
        if len(self.median_queue) < self.window_size:
            self.median_queue.append(x)
        else:
            self.median_queue.pop()
            self.median_queue.append(x)

        return median(self.median_queue)
    
class LowPassFilter:
    def __init__(self, loop_rate, time_constant):
        self.alpha = math.exp(-loop_rate / time_constant)
        self.prev_y = 0
        self.initialized = False

    def reset(self):
        self.prev_output = 0

    def update(self, x):
        if not self.initialized:
            new_y = x
            self.initialized = True
        else:
            new_y = self.alpha * self.prev_y + (1 - self.alpha) * x
            
        self.prev_y = new_y

        return new_y



        