import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from constants import MINIMUM_LOOKAHEAD_DISTANCE
import logging

logger =  logging.getLogger(__name__)

"""
A method which takes in the lane information for two lanes (current and next) and plots a path between them
"""

def plan_lane_change(lanes, current_lane):
    # TODO: Implement this. Maybe use somthing like a spline betweeen the two lanes?
    pass