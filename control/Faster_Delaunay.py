import numpy as np
import time
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from sklearn.feature_extraction import DictVectorizer


def separating(coordinates):
    coordinates.sort(key=lambda x: (x["Label"], x["Ypos"]**2+x["Xpos"]**2))
    dictvectorizer = DictVectorizer(dtype=np.float64, sparse=False)
    features = dictvectorizer.fit_transform(coordinates)
    final_data = np.delete(features, (0, 1, 2, 5), axis=1)
    return final_data


example = [{"Label": "Blue", "Zpos": 2, "Ypos": 3, "Xpos": 7, "Time": 1},
           {"Label": "Yellow", "Zpos": 2, "Ypos": 3.83, "Xpos": 8.26, "Time": 1},
           {"Label": "Yellow", "Zpos": 2, "Ypos": 5.15, "Xpos": 6.82, "Time": 1},
           {"Label": "Blue", "Zpos": 2, "Ypos": 4, "Xpos": 6, "Time": 1},
           {"Label": "Blue", "Zpos": 2, "Ypos": 4.49, "Xpos": 4.6, "Time": 1},
           {"Label": "Yellow", "Zpos": 2, "Ypos": 5.77, "Xpos": 5.18, "Time": 1}]
example.sort(key=lambda x: (x["Label"], x["Ypos"]**2+x["Xpos"]**2))
output = separating(example) 
tri = Delaunay(output)
final_triangles = tri.simplices
n = int(np.shape(final_triangles)[0])
for i in range(n):
    if example[tri.simplices[i][0]]["Label"] == example[tri.simplices[i][1]]["Label"] \
            == example[tri.simplices[i][0]]["Label"] == example[tri.simplices[i][2]]["Label"]:
        final_triangles = np.delete(final_triangles, i, 0)
plt.triplot(output[:, 0], output[:, 1], final_triangles)
plt.plot(output[:, 0], output[:, 1], 'o')
plt.show()
