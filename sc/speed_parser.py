import datetime
import logging

logger =  logging.getLogger(__name__)

class SpeedArray():
    """
    An input array for the system that tells Aeolus how fast to go at each part of the workout.
    Can be given in several formats including

    (time, speed): A series of durations and desired speeds (seconds, meters/s)
    (distance, speed): A series of distances and desired speeds (meters, meters/s)
    """

    def __init__(self, type, array):
        """
        param type: The format of the array
        param array: The array itself
        """
        self.type = type
        self.array = array

        self.startup_time = None
        self.current_index = 0

        self.final_array = []

        if self.type == 0:
            total_time = 0

            # Loop through the speed intervals and set their value as their end time
            for interval in self.array:
                total_time += interval[0]
                i_final_array = (total_time, interval[1])
                self.final_array.append(i_final_array)

            # Set the total time
            self.total_time = total_time

        elif self.type == 1:
            # TODO: Write this for the (distance, speed) type
            pass

    def set_startup_time(self, startup_time):
        """
        Sets the startup time

        param startup_time: When the workout was started
        """
        self.startup_time = startup_time

    def get_current_speed(self):
        """
        Gets the current speed by checking which speed interval we are in
        """
        # TODO: Write this for the other types
        # Get the current time
        current_time = datetime.datetime.now()

        # Calculate the total elapsed time
        total_elapsed_time = current_time - self.startup_time

        # logger.debug(f"SPEED PARSER: Elapsed time: {total_elapsed_time.total_seconds()}")

        # If the total elapsed time is greater than the total workout time, return the stop flag
        if total_elapsed_time.total_seconds() >= self.total_time:
            return 0, True

        # Loop through untill we reach the correct interval
        while self.current_index <= (len(self.final_array) - 1) and total_elapsed_time.total_seconds() > self.final_array[self.current_index][0]:
            self.current_index += 1

        if self.current_index == (len(self.final_array) - 1):
            return 0, True

        logger.debug(f"SPEED PARSER: Returning speed: {self.final_array[self.current_index][1]} for time: {total_elapsed_time.total_seconds()}")

        return self.final_array[self.current_index][1], False

            






    