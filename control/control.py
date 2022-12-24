from Faster_Delaunay import delauney_boundary
from path_finding import path_finding
from splines import generate_increment_on_path
# from math import atan
import numpy as np
from typing import List


def control(cones: List[dict], mode="autocross"):
    MAX_CLOSE_ORANGE_CONES_DISTANCE = 1

    oranges = []
    for cone in cones:
        if cone["Label"] == "Orange":
            cones.remove(cone)
            oranges.append(cone)

    if len(oranges) == 2:
        if np.sqrt((oranges[0][0] - oranges[1][0]) ** 2 + (oranges[0][1] - oranges[1][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:
            # we only have cones on one side
            oranges = np.average(oranges, axis=1)
        else:
            # the cones are on opposite sides
            ...
    elif len(oranges) == 3:
        if np.sqrt((oranges[0][0] - oranges[1][0]) ** 2 + (oranges[0][1] - oranges[1][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:
            oranges = [np.average(oranges[:2], axis=1), oranges[2]]
        elif np.sqrt((oranges[0][0] - oranges[2][0]) ** 2 + (oranges[0][1] - oranges[2][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:
            oranges = [np.average([oranges[0], oranges[2]], axis=1), oranges[1]]
        elif np.sqrt((oranges[2][0] - oranges[1][0]) ** 2 + (oranges[2][1] - oranges[1][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:
            oranges = [np.average(oranges[1:], axis=1), oranges[0]]
    elif len(oranges) == 4:
        new_oranges = []
        if np.sqrt((oranges[0][0] - oranges[1][0]) ** 2 + (oranges[0][1] - oranges[1][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:  # 0 - 1
            new_oranges.append(np.average([oranges[0], oranges[1]], axis=1))
        if np.sqrt((oranges[0][0] - oranges[2][0]) ** 2 + (oranges[0][1] - oranges[2][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:  # 0 - 2
            new_oranges.append(np.average([oranges[0], oranges[2]], axis=1))
        if np.sqrt((oranges[2][0] - oranges[1][0]) ** 2 + (oranges[2][1] - oranges[1][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:  # 2 - 1
            new_oranges.append(np.average([oranges[2], oranges[1]], axis=1))
        if np.sqrt((oranges[0][0] - oranges[3][0]) ** 2 + (oranges[0][1] - oranges[3][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:  # 0 - 3
            new_oranges.append(np.average([oranges[0], oranges[3]], axis=1))
        if np.sqrt((oranges[3][0] - oranges[2][0]) ** 2 + (oranges[3][1] - oranges[2][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:  # 3 - 2
            new_oranges.append(np.average([oranges[3], oranges[2]], axis=1))
        if np.sqrt((oranges[3][0] - oranges[2][0]) ** 2 + (oranges[3][1] - oranges[2][1]) ** 2) < \
                MAX_CLOSE_ORANGE_CONES_DISTANCE:  # 3 - 1
            new_oranges.append(np.average([oranges[3], oranges[1]], axis=1))

    triangles, cones = delauney_boundary(cones)
    midpoints = path_finding(triangles, cones)
    target_point = generate_increment_on_path(midpoints)
    theta = np.arctan(target_point[0] / target_point[1])

    return theta
