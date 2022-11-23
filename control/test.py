from Faster_Delaunay import delauney_boundary, separating
from path_finding import path_finding

example = [{"Label": "Blue", "Zpos": 2, "Ypos": 3, "Xpos": 7, "Time": 1},
           {"Label": "Yellow", "Zpos": 2, "Ypos": 3.83, "Xpos": 8.26, "Time": 1},
           {"Label": "Yellow", "Zpos": 2, "Ypos": 5.15, "Xpos": 6.82, "Time": 1},
           {"Label": "Blue", "Zpos": 2, "Ypos": 4, "Xpos": 6, "Time": 1},
           {"Label": "Blue", "Zpos": 2, "Ypos": 4.49, "Xpos": 4.6, "Time": 1},
           {"Label": "Yellow", "Zpos": 2, "Ypos": 5.77, "Xpos": 5.18, "Time": 1}]

triangles, cones = delauney_boundary(example)
midpoints = path_finding(triangles, cones)
print(midpoints)

