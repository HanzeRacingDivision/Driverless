from scipy.interpolate import splprep, splev
from scipy.signal import bspline
import numpy as np


def generate_increment_on_path(midpoints: np.ndarray, distance_increment: float = 0.05,
                               max_midpoints_considered: int = 3):
    """
    This function takes an array of midpoints and generates a new point along the path, described by these midpoints.
    If there is at least two midpoints, then we generate a cubic spline that fits the first few midpoints and our
    position (0,0). How many midpoints are considered is dependent on the argument max_midpoints_considered.
    If there is exactly one visible midpoint, we try to go there in a straight line.
    If there is no visible midpoint, we simply go straight. This case should never happen.

    :param midpoints: np.array([[x, y]]); the *ordered* list of midpoints (x and y coordinates) we want to follow
    :param distance_increment: float; default = 0.05; this determines how far along the spline a point is generated;
                               should depend on the upper bound of our speed and the lower bound of our pipeline latency
    :param max_midpoints_considered: int; default = 3; with the midpoint method, we mostly want to maneuver the closer
                                     midpoints and for that, taking further points into consideration when generating
                                     the cubic spline would decrease overall accuracy
    :return: np.array([x, y]); one pair of x-y-coordinates which represents the next point we want to steer towards
    """

    if midpoints.shape[0] > 2:
        # we only care about the first few midpoints to fit our line, and we add our current position to the array
        midpoints = np.concatenate([[[0, 0]], midpoints[: max_midpoints_considered]])

        mytck, myu = splprep([midpoints[:, 0], midpoints[:, 1]])
        xnew, ynew = splev([distance_increment], mytck)
        xnew, ynew = xnew[0], ynew[0]
    elif midpoints.shape[0] == 2:
        # x = a * (y ** 2) + b * y

        p1 = midpoints[0]
        p2 = midpoints[1]
        a = (p1[0] / p1[1] - p2[0] / p2[1]) / (p1[1] - p2[1])
        b = (p1[0] / p1[1]) - a * p1[1]

        ynew = distance_increment
        xnew = a * (ynew ** 2) + b * ynew
    elif midpoints.shape[0] == 1:
        # we move an incremental step on the line towards the point
        x, y = midpoints[1][0], midpoints[1][1]
        xnew = x * distance_increment
        ynew = y * distance_increment
    else:
        xnew, ynew = distance_increment, 0  # move straight

    return np.array([xnew, ynew])
