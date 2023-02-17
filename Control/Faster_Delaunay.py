import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from sklearn.feature_extraction import DictVectorizer


def separating(coordinates):
    """
    This function gets the list of dictionaries (containing the coordinates) and returns an array of [x-pos, y-pos]
    """

    # coordinates.sort(key=lambda x:(x["Label"], x["Ypos"]**2+x["Xpos"]**2))
    # dictvectorizer = DictVectorizer(dtype=np.float64, sparse=False)
    # features = dictvectorizer.fit_transform(coordinates)
    # final_data = np.delete(features, (0, 1, 2, 5), axis=1)
    final_data = np.ndarray((len(coordinates),2), dtype=np.float64)
    for i in range(len(coordinates)):
        final_data[i][0] = coordinates[i]["Xpos"]
        final_data[i][1] = coordinates[i]["Ypos"]
    return final_data


def delauney_boundary(coordinates):
    """
    This function recieves the dictionary, sorts it by colour and distance form the car (blue cones first and yellow cones after). 
    Then it outputs the delauney triangularization deleting that triangle that would be outside of the track.
    """

    coordinates.sort(key=lambda x: (x["Label"], x["Ypos"] ** 2 + x["Xpos"] ** 2))
    output = separating(coordinates)
    tri = Delaunay(output)
    final_triangles = tri.simplices
    n = int(np.shape(final_triangles)[0])
    n_deleted = 0
    for i in range(n):
        if coordinates[tri.simplices[i - n_deleted][0]]["Label"] == coordinates[tri.simplices[i - n_deleted][1]]["Label"] == \
                coordinates[tri.simplices[i - n_deleted][2]]["Label"]:
            final_triangles = np.delete(final_triangles, i - n_deleted, 0)
            n_deleted += 1

    return final_triangles, coordinates
