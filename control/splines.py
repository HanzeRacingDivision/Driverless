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
        ...
    else:
        xnew, ynew = distance_increment, distance_increment

    return np.array([xnew, ynew])
