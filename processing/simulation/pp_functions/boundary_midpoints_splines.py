from pygame.math import Vector2
import numpy as np
from scipy.interpolate import splprep, splev


def compute_boundaries(car,
                       left_cones,
                       right_cones,
                       visible_left_cones,
                       visible_right_cones,
                       first_left_cone_found,
                       first_right_cone_found,
                       new_visible_left_cone_flag,
                       new_visible_right_cone_flag,
                       left_spline_linked,
                       right_spline_linked,
                       track,
                       left_spline,
                       right_spline,
                       first_visible_left_cone,
                       first_visible_right_cone):
    
    # cubic splines for left track boundaries
    if len(visible_left_cones) > 1 and car.auto and new_visible_left_cone_flag:

        if not first_left_cone_found:
            first_visible_left_cone = visible_left_cones[0]
            first_left_cone_found = True
        
        x = []
        y = []
        for left_cone in visible_left_cones:
            x_temp = left_cone.true_position.x
            y_temp = left_cone.true_position.y
            
            x.append(x_temp)
            y.append(y_temp)
            
        if len(visible_left_cones) == 2:
            K = 1
        elif len(visible_left_cones) == 3:
            K = 2
        else:
            K = 2
            
        if len(visible_left_cones) == len(left_cones) and track == True and left_spline_linked == False:
            x.append(x[0])
            y.append(y[0])
            left_spline_linked = True

        tck, u = splprep([x,y], s=0, k = K)
        unew = np.arange(0, 1.01, 0.25/(len(visible_left_cones)**1.2))  # more cones = less final var
        left_spline = splev(unew, tck)
        
    
    # cubic splines for right track boundaries
    
    if len(visible_right_cones) > 1 and car.auto == True and new_visible_right_cone_flag == True:
        
        if not first_right_cone_found:
            first_visible_right_cone = visible_right_cones[0]
            first_right_cone_found = True
            
        x = []
        y = []
        for right_cone in visible_right_cones:
            x_temp = right_cone.true_position.x
            y_temp = right_cone.true_position.y
            
            x.append(x_temp)
            y.append(y_temp)
        
        if len(visible_right_cones) == 2:
            K = 1
        elif len(visible_right_cones) == 3:
            K = 2
                            
        else:
            K = 2
            
        if len(visible_right_cones) == len(right_cones) and track == True and right_spline_linked == False:
            x.append(x[0])
            y.append(y[0])
            right_spline_linked = True

        tck, u = splprep([x,y], s=0, k=K)
        unew = np.arange(0, 1.01, 0.25/(len(visible_right_cones)**1.2))  # more cones  = less final var
        right_spline = splev(unew, tck)
        
    return left_spline, right_spline, first_left_cone_found, left_spline_linked, first_right_cone_found,\
        right_spline_linked, first_visible_left_cone, first_visible_right_cone
    
    
def generate_midpoint_path(car,
                           Target,
                           targets,
                           non_passed_targets,
                           target_locations,
                           ppu,
                           visible_left_cones,
                           visible_right_cones,
                           car_angle,
                           new_visible_left_cone_flag,
                           new_visible_right_cone_flag):

    # auto generate path based on splines/cones
    
    if (len(visible_left_cones) > 1 and len(visible_right_cones) > 1
            and (new_visible_right_cone_flag or new_visible_left_cone_flag)):  # track_number == 0 and

        path_midpoints_x = []  # [car.position.x]
        path_midpoints_y = []  # [car.position.y]
        
        for left_cone in visible_left_cones:
            for right_cone in visible_right_cones:
                if np.linalg.norm((left_cone.true_position.x - right_cone.true_position.x,
                                   left_cone.true_position.y - right_cone.true_position.y)) < 4:
                    path_midpoints_x.append(np.mean([left_cone.true_position.x, right_cone.true_position.x]))
                    path_midpoints_y.append(np.mean([left_cone.true_position.y, right_cone.true_position.y]))
                    
        path_midpoints = [path_midpoints_x, path_midpoints_y]
        
        path_midpoints_visible_x = []  
        path_midpoints_visible_y = []      
        path_to_sort = []
        
        # find all 'visible' midpoints - this may be inefficient
        for i in range(len(path_midpoints[0])):
            # print(i)
            dist_car = np.linalg.norm(Vector2(path_midpoints[0][i],path_midpoints[1][i]) - car.true_position)
    
            # calculating angle between car angle and midpoint
            if dist_car < car.fov/ppu:
                
                a_b = Vector2(path_midpoints[0][i],path_midpoints[1][i]) - car.true_position
                a_b = np.transpose(np.matrix([a_b.x,-1*a_b.y ]))
                
                rotate = np.matrix([[np.cos(-car_angle*np.pi/180),-1*np.sin(-car_angle*np.pi/180)],
                                    [np.sin(-car_angle*np.pi/180),np.cos(-car_angle*np.pi/180)]])
                
                a_b = rotate*a_b
                
                a = a_b[0]
                b = a_b[1]
                
                beta = np.arctan(b/a)*(180/np.pi)
                alpha = beta + 90*(b/np.abs(b))*np.abs((a/np.abs(a)) - 1)
                alpha = alpha[0,0]
                
                # if target within car fov, set to visible
                if np.abs(alpha) < car.fov_range:
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
            path_midpoints = [[car.true_position.x , path_midpoints[0][0]], [car.true_position.y, path_midpoints[1][0]]]
            
            tck, u = splprep(path_midpoints, s=1, k = 1)
            unew = np.arange(0, 1.01, 0.5/(len(visible_left_cones)**0.4))  # more cones  = less final var
            path_midpoints_spline = splev(unew, tck)
            
        elif len(path_midpoints[0]) == 2:
            tck, u = splprep(path_midpoints, s=1, k = 1)
            unew = np.arange(0, 1.01, 0.5/(len(visible_left_cones)**0.4))  # more cones  = less final var
            path_midpoints_spline = splev(unew, tck)
            
        elif len(path_midpoints[0]) > 2:
            tck, u = splprep(path_midpoints, s=1, k = 2)
            unew = np.arange(0, 1.01, 0.5/(len(visible_left_cones)**0.4))  # more cones  = less final var
            path_midpoints_spline = splev(unew, tck)

        else:
            path_midpoints_spline = []
            
        if len(path_midpoints_spline) > 0:
            for i in range(len(path_midpoints_spline[0])):
                new_target_loc = [path_midpoints_spline[0][i], path_midpoints_spline[1][i]]
                if new_target_loc in target_locations:
                    continue
                else:
                    make_target = True
                    for j in range(len(target_locations)):
                        if np.linalg.norm(tuple(x-y for x,y in zip(target_locations[j],new_target_loc))) < 1:
                            make_target = False
                            break
                    
                    if make_target:
                        new_target = Target(new_target_loc[0], new_target_loc[1])
                        targets.append(new_target)
                        non_passed_targets.append(new_target)
                        target_locations.append(new_target_loc)
                        
    return targets, non_passed_targets, target_locations
