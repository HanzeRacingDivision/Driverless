import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from sklearn.feature_extraction import DictVectorizer


def separating(coordinates):
    # coordinates.sort(key=lambda x:(x["Label"], x["Ypos"]**2+x["Xpos"]**2))
    dictvectorizer = DictVectorizer(dtype=np.float64, sparse=False)
    features = dictvectorizer.fit_transform(coordinates)
    final_data = np.delete(features, (0, 1, 2, 5), axis=1)
    return final_data


def delauney_boundary(coordinates):
    coordinates.sort(key=lambda x: (x["Label"], x["Ypos"]**2+x["Xpos"]**2))
    output = separating(coordinates)
    tri = Delaunay(output)
    final_triangles = tri.simplices
    n = int(np.shape(final_triangles)[0])
    for i in range(n):
        if coordinates[tri.simplices[i][0]]["Label"] == coordinates[tri.simplices[i][1]]["Label"] == coordinates[tri.simplices[i][2]]["Label"]:
            final_triangles = np.delete(final_triangles, i, 0)

    return final_triangles, coordinates
