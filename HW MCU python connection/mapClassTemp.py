import numpy as np

def deepCopy(source): #can copy any list, variable or class (copy values, not pointers)
    #print(type(source))
    if(source.__class__.__module__ == 'builtins'): #python builtin types, like <int> and <list> and such
        if((type(source) is list) or (type(source) is bytearray)):
            returnList = source.__class__() #make new list/bytearray (same class as source)
            for entry in source:
                returnList.append(deepCopy(entry))
            return(returnList)
        else:
            return(source)
    else: #custom classes (__module__ returns something like '__main__')
        returnObject = source.__class__() #make new instance of same class as source
        classAttributs = dir(source) #returs a list of all class attributes
        # alternatively, you might be able to retrieve an attribute name list with: getattr(getattr(getattr(source, '__init__'), '__code__'), 'co_names')
        for entry in classAttributs:
            if((not entry.startswith('_')) and (not callable(getattr(source, entry)))): #if the attribute is not private (low level stuff) or a function (method)
                setattr(returnObject, entry, deepCopy(getattr(source, entry))) #copy attribute
        return(returnObject)

class Map:
    """ A Parent Map Class that SLAM, PathPlanning and other simulations inherit from """

    def __init__(self, carStartPos=[0,0]):  # variables here that define the scenario/map
        self.car = self.Car(carStartPos[0], carStartPos[1])
        self.left_cone_list = []
        self.right_cone_list = []
        self.finish_line_cones = [] #holds 2 cones, 1 left and 1 right

    class Car:
        def __init__(self, x=0, y=0, angle=0, max_steering=80, max_acceleration=4.0):
            self.position = [x, y]
            self.velocity = 0.0
            self.angle = angle
            self.length = 2
            self.width = 1
            self.max_acceleration = max_acceleration
            self.max_steering = max_steering
            self.max_velocity = 5

            self.acceleration = 0.0
            self.steering = 0.0
            self.fov_range = 60  #(thijs) this is the actual (camera) field of view variable, but it's only useful for the simulation, so delete this?
            self.auto = False #(thijs) needs a clearer name like 'driving' or 'self_driving_active' or something
            
            self.coneConData = None #coneConDataClass() #data for cone-connection
            self.pathFolData = None #pathFolDataClass() #data for path-following
            self.slamData = None #slamDataClass() #data for SLAM

        def update(self, dt):
            self.velocity += self.acceleration * dt
            self.velocity = max(-self.max_velocity, min(self.velocity, self.max_velocity))

            if self.steering:
                turning_radius = self.length / np.sin(np.radians(self.steering))
                angular_velocity = self.velocity / turning_radius
            else:
                angular_velocity = 0

            self.position[0] += dt * self.velocity * np.cos(self.angle)
            self.position[1] += dt * self.velocity * np.sin(self.angle)
            self.angle += np.degrees(angular_velocity) * dt

    class Cone:
        """ Class for storing the coordinates and visibility of each cone """
        def __init__(self, x=0, y=0, leftOrRight=False, isFinish=False):
            self.position = [x, y]
            self.LorR = leftOrRight #boolean to indicate which side of the track (which color) the code is
            self.isFinish = isFinish
            
            self.coneConData = None #coneConDataClass() #data for cone-connection
            self.pathPlanData = None #pathPlanDataClass() #data for path-planning
            self.slamData = None #slamDataClass() #data for SLAM

    class Target:
        """ 'Target' child class only used in PathPlanning """
        def __init__(self, x=0, y=0):
            self.position = [x, y]
            self.passed = 0 #counts the number of times it's been passed
            
            self.coneConData = None #coneConDataClass() #data for cone-connection
            self.pathPlanData = None #pathPlanDataClass() #data for path-planning
            self.slamData = None #slamDataClass() #data for SLAM


def get_angle_between(obj_1, obj_2, obj_2_angle):
    return(np.arctan2(obj_2[1]-obj_1[1], obj_2[0]-obj_1[0])-obj_2_angle)
