from scipy.interpolate import splprep, splev
import numpy as np


def generate_increment_on_path(midpoints: np.ndarray, distance_increment: float = 0.05,
                               max_midpoints_considered: int = 3):
    """
    This function takes an array of midpoints and generates a new point along the path, described by these midpoints.

    # TODO Do we maybe need an absolute increment (or one dependent on our speed)?
    Currently, our 'step size' is dependent on how far the last midpoint we consider, is away.

    :param midpoints:
    :param distance_increment: float
    :param max_midpoints_considered: int
    :return:
    """

    if midpoints.shape[0] >= 2:
        # we only care about the first few midpoints to fit our line, and we add our current position to the array
        midpoints = np.concatenate([[[0, 0]], midpoints[: max_midpoints_considered]])

        mytck, myu = splprep([midpoints[:, 0], midpoints[:, 1]])
        xnew, ynew = splev([distance_increment], mytck)
        xnew, ynew = xnew[0], ynew[0]
    elif midpoints.shape[0] == 1:
        # we move an incremental step on the line towards the point
        x, y = midpoints[1][0], midpoints[1][1]
        xnew = x * distance_increment
        ynew = y * distance_increment
    else:
        xnew, ynew = distance_increment, 0  # move straight

    return np.array([xnew, ynew])
