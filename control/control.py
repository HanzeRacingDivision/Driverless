from Faster_Delaunay import delauney_boundary
from path_finding import path_finding
from splines import generate_increment_on_path
from math import atan


def control(cones):
    triangles, cones = delauney_boundary(cones)
    midpoints = path_finding(triangles, cones)
    target_point = generate_increment_on_path(midpoints)
    theta = atan(target_point[0] / target_point[1])

    return theta
