import numpy as np
from typing import List


def path_finding(triangles: np.ndarray, cones: List[dict]):
    """
    This function will take an array of edges between cones of opposite color and first find the order in which they are
    positioned on the track. Then, it will determine the midpoints between of the edges and return them as an ordered
    list, with the first element being the closest midpoint.

    :param cones: ordered list of cones (ordered by colour and then distance to car)
    :param triangles: np.ndarray([n1, n2, n3])
    :return midpoints: np.ndarray[p]
    """

    edges = []
    for triangle in triangles:
        if cones[triangle[0]]["Label"] != cones[triangle[1]]["Label"]:
            edges.append([[cones[triangle[0]]["Xpos"], cones[triangle[0]]["Ypos"]],
                          [cones[triangle[1]]["Xpos"], cones[triangle[1]]["Ypos"]]])
        if cones[triangle[2]]["Label"] != cones[triangle[1]]["Label"]:
            edges.append([[cones[triangle[2]]["Xpos"], cones[triangle[2]]["Ypos"]],
                          [cones[triangle[1]]["Xpos"], cones[triangle[1]]["Ypos"]]])
        if cones[triangle[2]]["Label"] != cones[triangle[0]]["Label"]:
            edges.append([[cones[triangle[2]]["Xpos"], cones[triangle[2]]["Ypos"]],
                          [cones[triangle[0]]["Xpos"], cones[triangle[0]]["Ypos"]]])
        if cones[triangle[0]]["Label"] == cones[triangle[1]]["Label"] == cones[triangle[2]]["Label"] == "Orange":
            ...

    for i in range(len(edges)):
        edges[i] = sorted(edges[i], key=lambda x: x[0]**2 + x[1]**2)

    edges = np.array(edges)
    p1s = edges[:, 0]
    distances = list(np.sum(p1s, axis=1))
    idx = distances.index(min(distances))
    current_edge = edges[idx]
    ordered_edges = np.ndarray(edges.shape)
    ordered_edges[0] = current_edge
    for i in range(1, edges.shape[0]):
        for j in range(edges.shape[0]):
            if np.all(edges[j][0] == current_edge[1]):
                current_edge = edges[j]
                ordered_edges[i] = current_edge
                break

    midpoints = np.average(ordered_edges, axis=1)

    return midpoints
