from scipy.optimize import fsolve
import numpy as np


# We need them sorted by colours: first blue is given, after yellow (as we already do in the delauney triangularization)

path = 'clockwise'

coordinates = [{"Label": "Blue", "Zpos": None, "Ypos": 6.9265658398279, "Xpos": 8.0568797103177, "Time": None}, {"Label": "Yellow", "Zpos": None, "Ypos": 4.6831656610596, "Xpos": 6.0173569273455, "Time": None}]

if path == 'clockwise': # in this case blue is outside and yellow inside
    if coordinates[0]["Label"] == coordinates[1]["Label"] == "Yellow":
        r_1 = r_2 = 7.625
    elif coordinates[0]["Label"] == coordinates[1]["Label"] == "Blue":
        r_1 = r_2 = 10.625
    else:
        r_1 = 7.625 
        r_2 = 10.625

    x1_pos = coordinates[1]["Xpos"]
    y1_pos = coordinates[1]["Ypos"]
    x2_pos = coordinates[0]["Xpos"]
    y2_pos = coordinates[0]["Ypos"]

else:# in this case blue is inside and yellow outside
    if coordinates[0]["Label"] == coordinates[1]["Label"] == "Yellow":
        r_1 = r_2 = 10.625
    elif coordinates[0]["Label"] == coordinates[1]["Label"] == "Blue":
        r_1 = r_2 = 7.625 
    else:
        r_1 = 7.625
        r_2 = 10.625

    x1_pos = coordinates[0]["Xpos"]
    y1_pos = coordinates[0]["Ypos"]
    x2_pos = coordinates[1]["Xpos"]
    y2_pos = coordinates[1]["Ypos"]


# print(x1_pos,x2_pos,y1_pos,y2_pos,r_1,r_2) (IT WORKS!!!)

def equations(vars,x1_pos,y1_pos,x2_pos,y2_pos,): # (x,y) are the coordinates of the center of the circle
    x,y = vars
    eq1 = x**2-2*(x1_pos)*x+(x1_pos)**2+y**2-2*(y1_pos)*y+(y1_pos)**2-(r_1)**2
    eq2 = x**2-2*(x2_pos)*x+(x2_pos)**2+y**2-2*(y2_pos)*y+(y2_pos)**2-(r_2)**2
    return [eq1, eq2]

h, k =  fsolve(equations,(0.5, 0.5), args = (x1_pos,y1_pos,x2_pos,y2_pos))

print(np.single(h),np.single(k))
