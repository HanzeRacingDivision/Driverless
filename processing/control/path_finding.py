import numpy as np


def path_finding(edges: np.ndarray):
    """
    This function will take an array of edges between cones of opposite color and first find the order in which they are
    positioned on the track. Then, it will determine the midpoints between of the edges and return them as an ordered
    list, with the first element being the closest midpoint.

    :param edges: np.ndarray([[p1, p2]])
    :return midpoints: np.ndarray[p]
    """

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
