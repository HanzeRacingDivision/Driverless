import numpy as np
from typing import List


def path_finding(triangles: np.ndarray, cones: List[dict]):
    """
    This function will take a list of triangles between cones, generated by Delaunay triangularization. This is
    converted to an array of edges between cones of opposite color. Then it first finds the order in which they
    are positioned on the track. Secondly, it will determine the midpoints between of the edges and return them
    as an ordered list, with the first element being the first upcoming midpoint.

    :param triangles: np.ndarray([n1, n2, n3]); n1, n2 and n3 is integers; numbers represent position in the list cones
    :param cones: ordered list of dictionaries describing cones (ordered by colour and then distance to car);
                  dictionary key-value pairs:
                    "Label": str ("Yellow" or "Blue")
                    "Xpos": float
                    "Ypos": float
    :return midpoints: np.ndarray[[x, y]]; x and y as floats
    """

    midpoints = []
    for triangle in triangles:
        if cones[triangle[0]]["Label"] != cones[triangle[1]]["Label"]:
            midpoints.append([(cones[triangle[0]]["Xpos"] + cones[triangle[1]]["Xpos"]) / 2,
                             (cones[triangle[0]]["Ypos"] + cones[triangle[1]]["Ypos"]) / 2])
        if cones[triangle[2]]["Label"] != cones[triangle[1]]["Label"]:
            midpoints.append([(cones[triangle[2]]["Xpos"] + cones[triangle[1]]["Xpos"]) / 2,
                             (cones[triangle[2]]["Ypos"] + cones[triangle[1]]["Ypos"]) / 2])
        if cones[triangle[2]]["Label"] != cones[triangle[0]]["Label"]:
            midpoints.append([(cones[triangle[2]]["Xpos"] + cones[triangle[0]]["Xpos"]) / 2,
                              (cones[triangle[2]]["Ypos"] + cones[triangle[0]]["Ypos"]) / 2])

    unique_midpoints = []
    for midpoint in midpoints:
        if midpoint not in unique_midpoints:
            unique_midpoints.append(midpoint)

    midpoints = unique_midpoints

    if len(midpoints) == 0:
        return np.array([])

    distances = [p[0]**2 + p[1]**2 for p in midpoints]
    idx = distances.index(min(distances))
    current_midpoint = midpoints[idx]
    used_indexes = [idx]
    ordered_midpoints = [current_midpoint]
    for _ in range(1, len(midpoints)):
        next_idx = None
        next_distance = np.infty
        for j in range(len(midpoints)):
            if j in used_indexes:
                continue
            t = np.sqrt((midpoints[j][0] - current_midpoint[0]) ** 2 + (midpoints[j][1] - current_midpoint[1]) ** 2)
            if t < next_distance:
                next_distance = t
                next_idx = j
        if next_idx is not None:
            current_midpoint = midpoints[next_idx]
            ordered_midpoints.append(current_midpoint)
            used_indexes.append(next_idx)

    return np.array(ordered_midpoints)
