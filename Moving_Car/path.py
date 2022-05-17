from pygame.math import Vector2
import numpy as np
from scipy.interpolate import splprep, splev

from cone import Side
from target import Target


class Path:
    def __init__(self):
        self.spline_image = {Side.LEFT: None, Side.RIGHT: None}

        self.splines = {Side.LEFT: 0, Side.RIGHT: 0}
        self.spline_linked = {Side.LEFT: False, Side.RIGHT: False}

        self.start_midpoint_x = 0
        self.start_midpoint_y = 0

    def compute_boundaries(self, pp):

        # cubic splines for left track boundaries
        for category in Side:
            if len(pp.cones.visible[category]) > 1 and pp.car.auto and pp.cones.new_visible_cone_flag[category]:
                if not pp.cones.first_cone_found[category]:
                    pp.cones.first_visible_cone[category] = pp.cones.visible[category][0]
                    pp.cones.first_cone_found[category] = True

                x = []
                y = []
                for cone in pp.cones.visible[category]:
                    x_temp = cone.position.x
                    y_temp = cone.position.y

                    x.append(x_temp)
                    y.append(y_temp)

                if len(pp.cones.visible[category]) == 2:
                    K = 1
                else:
                    K = 2

                if len(pp.cones.visible[category]) == len(pp.cones.list[category]) and not \
                        self.spline_linked[category] and pp.track:
                    x.append(x[0])
                    y.append(y[0])
                    self.spline_linked[category] = True

                tck, u = splprep([x, y], s=0, k=K)
                unew = np.arange(0, 1.01, 0.25 / (
                        len(pp.cones.visible[category]) ** 1.2))  # more cones  = less final var
                self.splines[category] = splev(unew, tck)

    def generate_midpoint_path(self, pp):

        # auto generate path based on splines/cones
        if (len(pp.cones.in_fov[Side.LEFT]) > 1
                and len(pp.cones.in_fov[Side.RIGHT]) > 1):
                #and (pp.cones.new_visible_cone_flag[Side.LEFT] or pp.cones.new_visible_cone_flag[Side.RIGHT])):  # track_number == 0 and
            path_midpoints_x = []  # [car.position.x]
            path_midpoints_y = []  # [car.position.y]

            for left_cone in pp.cones.in_fov[Side.LEFT]:
                for right_cone in pp.cones.in_fov[Side.RIGHT]:
                    if np.linalg.norm((left_cone.position.x - right_cone.position.x,
                                       left_cone.position.y - right_cone.position.y)) < 4:
                        path_midpoints_x.append(np.mean([left_cone.position.x, right_cone.position.x]))
                        path_midpoints_y.append(np.mean([left_cone.position.y, right_cone.position.y]))

            path_midpoints = [path_midpoints_x, path_midpoints_y]

            path_midpoints_visible_x = []
            path_midpoints_visible_y = []
            path_to_sort = []

            # couple each midpoint with its distance to the car
            for i in range(len(path_midpoints[0])):
                dist_car = np.linalg.norm(Vector2(path_midpoints[0][i], path_midpoints[1][i]) - pp.car.position)
                path_to_sort.append([dist_car, path_midpoints[0][i], path_midpoints[1][i]])

            # ordering the path_midpoints by distance from car
            path_to_sort.sort()
            if len(path_to_sort) > 1:
                for i in range(len(path_to_sort)):
                    # if statement making sure we have no duplicate co-ordinates
                    if path_to_sort[i][1] in path_midpoints_visible_x or path_to_sort[i][2] in path_midpoints_visible_y:
                        pass
                    else:
                        path_midpoints_visible_x.append(path_to_sort[i][1])
                        path_midpoints_visible_y.append(path_to_sort[i][2])

            path_midpoints_visible = [path_midpoints_visible_x, path_midpoints_visible_y]
            # path_midpoints_visible.sort()
            path_midpoints = path_midpoints_visible

            if len(path_midpoints[0]) == 1:
                path_midpoints = [[pp.car.position.x, path_midpoints[0][0]], [pp.car.position.y, path_midpoints[1][0]]]

                tck, u = splprep(path_midpoints, s=1, k=1)
                unew = np.arange(0, 1.01, 0.5 / (
                        len(pp.cones.visible[Side.LEFT]) ** 0.4))  # more cones  = less final var
                path_midpoints_spline = splev(unew, tck)

            elif len(path_midpoints[0]) == 2:
                tck, u = splprep(path_midpoints, s=1, k=1)
                unew = np.arange(0, 1.01, 0.5 / (
                        len(pp.cones.visible[Side.LEFT]) ** 0.4))  # more cones  = less final var
                path_midpoints_spline = splev(unew, tck)

            elif len(path_midpoints[0]) > 2:
                tck, u = splprep(path_midpoints, s=1, k=2)
                unew = np.arange(0, 1.01, 0.5 / (
                        len(pp.cones.visible[Side.LEFT]) ** 0.4))  # more cones  = less final var
                path_midpoints_spline = splev(unew, tck)

            else:
                path_midpoints_spline = []

            if len(path_midpoints_spline) > 0:
                for i in range(len(path_midpoints_spline[0])):
                    new_target_loc = [path_midpoints_spline[0][i], path_midpoints_spline[1][i]]
                    if new_target_loc in pp.targets.target_locations:
                        continue
                    else:
                        make_target = True
                        for j in range(len(pp.targets.target_locations)):
                            if np.linalg.norm(
                                    tuple(x - y for x, y in zip(pp.targets.target_locations[j], new_target_loc))) < 1:
                                make_target = False
                                break

                        if make_target:
                            new_target = Target(new_target_loc[0], new_target_loc[1])
                            pp.targets.targets.append(new_target)
                            pp.targets.non_passed_targets.append(new_target)
                            pp.targets.target_locations.append(new_target_loc)
