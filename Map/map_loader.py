import pandas as pd
import os
import datetime     #used for naming files automatically
from Map import Map

def copyImportMap(classWithMapParent, mapToImport):
    """save all attributes from mapToImport to those by the same name in classWithMapParent"""
    for attrName in dir(mapToImport): #dir(class) returs a list of all class attributes
        if((not attrName.startswith('_')) and (not callable(getattr(mapToImport, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
            setattr(classWithMapParent, attrName, getattr(mapToImport, attrName)) #copy attribute

# from copy import deepcopy #only used in copyExtractMap
# def deepCopyExtractMap(classWithMapParent):
#     """copy ONLY the map class attributes from any (child) class into a new map object"""
#     returnObject = Map() #make new instance of same class as source
#     for attrName in dir(returnObject): #dir(class) returs a list of all class attributes
#         if((not attrName.startswith('_')) and (not callable(getattr(returnObject, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
#             setattr(returnObject, attrName, deepcopy(getattr(classWithMapParent, attrName))) #deepcopy attribute
#     return(returnObject)

# def shallowCopyExtractMap(classWithMapParent):
#     """copy ONLY the map class attributes from any (child) class into a 'new' map object (consisting of pointers)"""
#     returnObject = Map() #make new instance of same class as source
#     for attrName in dir(returnObject): #dir(class) returs a list of all class attributes
#         if((not attrName.startswith('_')) and (not callable(getattr(returnObject, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
#             setattr(returnObject, attrName, getattr(classWithMapParent, attrName)) #copy attribute (pointer)
#     return(returnObject)


class mapLoader:
    fileExt = ".xlsx" #static
    def __init__(self, immediateFile=None, immediateLoadTarget=None):
        self.defaultFilename = "map_"
        if(immediateFile is not None):
            print("loading map at startup:", immediateFile)
            if(immediateLoadTarget is not None):
                print("loading startup map into self")
                immediateLoadTarget = self
            self.laod_map(immediateFile, immediateLoadTarget)
    
    def mapObjectToFile(self, mapToSave):
        combinedConeList = mapToSave.left_cone_list + mapToSave.right_cone_list
        map_file = pd.DataFrame({'Cone_Type' : [('RIGHT' if cone.LorR else 'LEFT') for cone in combinedConeList],
                                 'Cone_X' : [cone.position[0] for cone in combinedConeList],
                                 'Cone_Y' : [cone.position[1] for cone in combinedConeList],
                                 'Finish' : [cone.isFinish for cone in combinedConeList]})
        return(map_file)
    
    def mapFileToObject(self, map_file):
        mapObj = Map()
        for row in range(len(map_file)):
            coneToCovert = map_file.iloc[row] #load the row
            mapCone = Map.Cone(row, [coneToCovert['Cone_X'], coneToCovert['Cone_Y']], (coneToCovert['Cone_Type'] == 'RIGHT'), coneToCovert['Finish'])
            listToAppend = (mapObj.right_cone_list if mapCone.LorR else mapObj.left_cone_list)
            listToAppend.append(mapCone)
        return(mapObj)
    
    def save_map(self, mapToSave: Map, filename=None):
        if(filename is None): #automatic file naming
            filename = self.defaultFilename + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S") + self.fileExt
        elif(not filename.endswith(self.fileExt)):
            filename += self.fileExt
        print("saving to file:", filename)
        #print(map_file)
        map_file = self.mapObjectToFile(mapToSave)
        map_file.to_excel(filename)
    
    def laod_map(self, filename, whereToLoad=None):
        if(not filename.endswith(self.fileExt)):
            filename += self.fileExt
        current_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = filename #init var
        if(not filename.startswith(current_dir)):
             map_path = os.path.join(current_dir, filename)
        map_file = pd.read_excel(map_path, index_col=0) #'index_col=0' lets the parser know that the indices are in the first column (and so the value in header row is unimportant)
        print("loading map file:", map_path)
        #print(map_file)
        returnMap = self.mapFileToObject(map_file)
        if(whereToLoad is not None):
            copyImportMap(whereToLoad, returnMap)
        return(returnMap)


if __name__ == '__main__':
    loader = mapLoader()
    aMap = Map()
    aMap.left_cone_list.append(Map.Cone(1, [0.25, 0.75], False, False))
    aMap.right_cone_list.append(Map.Cone(2, [0, 0.123456789], True, True))
    loader.save_map(aMap, "test")
    returnedMap = loader.laod_map("test")
