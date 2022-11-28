from scipy.interpolate import splprep, splev
import numpy as np


def generate_spline(midpoints: np.ndarray, distance_increment: float = 0.05):
    midpoints = np.concatenate([[[0, 0]], midpoints])
    if midpoints.shape[0] >= 3:
        midpoints = midpoints[:3]

        mytck, myu = splprep([midpoints[:, 0], midpoints[:, 1]])
        xnew, ynew = splev([distance_increment], mytck)
        xnew, ynew = xnew[0], ynew[0]
    elif midpoints.shape[0] == 2:
        x1, y1, x2, y2 = midpoints[0][0], midpoints[0][1], midpoints[1][0], midpoints[1][1]
        xnew = x1 + (x2 - x1) * distance_increment
        ynew = y1 + (y2 - y1) * distance_increment
    else:
        xnew, ynew = distance_increment, distance_increment

    return np.array([xnew, ynew])
