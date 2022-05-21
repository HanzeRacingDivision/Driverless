import pandas as pd
import os

cone_connect_list = False  # variable for whether the order of the cones is defined in a variable


def save_map(left_cones, right_cones):
    cone_x = []
    cone_y = []
    cone_type = []
    print('SAVE MAP AS : ')
    name = input()

    for i in range(len(left_cones)):
        cone_x.append(left_cones[i].true_position.x)
        cone_y.append(left_cones[i].true_position.y)
        cone_type.append('LEFT')

    for i in range(len(right_cones)):
        cone_x.append(right_cones[i].true_position.x)
        cone_y.append(right_cones[i].true_position.y)
        cone_type.append('RIGHT')

    if cone_connect_list:
        map_file = pd.DataFrame({'Cone_Type': cone_type,
                                 'Cone_X': cone_x,
                                 'Cone_Y': cone_y,
                                 'Prev_Cone_Index': prev_cone_index,
                                 'Next_Cone_Index': next_cone_index})
    else:
        map_file = pd.DataFrame({'Cone_Type': cone_type,
                                 'Cone_X': cone_x,
                                 'Cone_Y': cone_y})

    map_file.to_csv(f'{name}.csv')


def load_map(mouse_pos_list):
    left_cones = []
    right_cones = []
    print('LOAD MAP : ')
    name = input()

    current_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(current_dir, f"{name}.csv")
    map_file = pd.read_csv(map_path)

    for i in range(len(map_file.iloc[:, 0])):
        if map_file['Cone_Type'].iloc[i] == 'LEFT':

            left_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], 'left')
            left_cones.append(left_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i] * ppu, map_file['Cone_Y'].iloc[i] * ppu))

        else:
            right_cone = Cone(map_file['Cone_X'].iloc[i], map_file['Cone_Y'].iloc[i], 'right')
            right_cones.append(right_cone)
            mouse_pos_list.append((map_file['Cone_X'].iloc[i] * ppu, map_file['Cone_Y'].iloc[i] * ppu))

    return left_cones, right_cones, mouse_pos_list
