from control.Faster_Delaunay import delauney_boundary
from control.path_finding import path_finding
from control.splines import generate_increment_on_path
from math import atan


def control(cones):
    triangles, cones = delauney_boundary(cones)
    midpoints = path_finding(triangles, cones)
    target_point = generate_increment_on_path(midpoints, distance_increment=0.1, max_midpoints_considered=6)
    theta = atan(target_point[0] / target_point[1])
    return theta
