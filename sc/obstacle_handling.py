from core.sc.filters import MedianFilter, MovingAverageFilter
from core.constants import STOP_TIME_THRESHOLD

"""
Goal: Detect obstacles in the path of the vehicle. If the obstacle is far enough away, change lanes to avoid it. If the obstacle is too close, stop the vehicle.

Details:
    * Filtering: Apply some noise filtering to the obstacle distance filter to get a more accurate reading and prevent false postives (low pass?)
    * Lane change: If the obstacle is within a certain distance threshold (modulated based on speed), plan and execute a lane change
    * If the obstacle is within a safety buffer (modulated by speed), immediately brake

    * Returns two booleans, lane change, stop

Structure
    * Process that gets called after state estimation
    * Takes in most recent state
    * Outputs some sort of decision data structure
        * Normal/Lane-Change/Stop
        * Gets checked before sending speed command
"""

OBSTACLE_MEDIAN_FILTER_WINDOW_SIZE = 30 # TODO: Decide whether to move constants
OBSTACLE_MOVING_AVERAGE_WINDOW_SIZE = 10
DISTANCE_THRESHOLD = 0.2


class ObstacleHandler:

    def __init__(self):
        self.median_filter = MedianFilter(OBSTACLE_MEDIAN_FILTER_WINDOW_SIZE)
        self.average_filter = MovingAverageFilter(OBSTACLE_MOVING_AVERAGE_WINDOW_SIZE)


    def update_chained_filter(self, distance):

        medianed = self.median_filter.update(distance)
        averaged = self.average_filter.update(medianed)

        return averaged

    def update_handler(self, vehicle_state):

        distance = vehicle_state.ultra_distance
        velocity = vehicle_state.velocity

        filtered_distance = self.update_chained_filter(distance)
        obstacle_detected = filtered_distance < DISTANCE_THRESHOLD

        stop = False
        change_lane = True

        if obstacle_detected:
            if (filtered_distance / velocity) < STOP_TIME_THRESHOLD:
                stop = True
            else:
                # TODO: Put lane change code here
                pass
        
        return stop
            




        