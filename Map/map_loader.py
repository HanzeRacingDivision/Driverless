import pandas as pd
import numpy as np
import os
import datetime     #used for naming files automatically
from Map import Map

import generalFunctions as GF


class mapLoader:
    """a class for saving/loading/importing pandas excel files
        (currently!) only saves cone states (positions, connections, etc)"""
    fileExt = ".xlsx" #static
    def __init__(self, immediateFile=None, immediateLoadTarget=None):
        self.defaultFilename = "map_"
        
        ##these should probably be centralized in Map.py, but they are used here, so frick it. Initialize to False, and set to True in child class (instance that combines the component and Map classes)
        self.coneConnecterPresent = False
        self.pathFinderPresent = False
        self.pathPlanningPresent = False
        self.SLAMPresent = False
        
        if(immediateFile is not None):
            print("loading map at startup:", immediateFile)
            if(immediateLoadTarget is None):
                print("loading startup map into self")
                immediateLoadTarget = self
            self.laod_map(immediateFile, immediateLoadTarget)
    
    def mapObjectToFile(self, mapToSave):
        """convert Map object to pandas dataframe"""
        combinedConeList = mapToSave.left_cone_list + mapToSave.right_cone_list
        map_file = pd.DataFrame({'Cone_Type' : [('RIGHT' if cone.LorR else 'LEFT') for cone in combinedConeList],
                                 'Cone_X' : [cone.position[0] for cone in combinedConeList],
                                 'Cone_Y' : [cone.position[1] for cone in combinedConeList],
                                 'Finish' : [cone.isFinish for cone in combinedConeList],
                                 'Conn_A' : [(GF.findIndexByClassAttr(combinedConeList, 'ID', cone.connections[0].ID) if (len(cone.connections)>0) else None) for cone in combinedConeList],
                                 'Conn_B' : [(GF.findIndexByClassAttr(combinedConeList, 'ID', cone.connections[1].ID) if (len(cone.connections)>1) else None) for cone in combinedConeList]})
        return(map_file)
    
    def mapFileToObject(self, map_file):
        """convert pandas dataframe (map_file) to Map object"""
        mapObj = Map()
        for row in range(len(map_file)):
            coneToCovert = map_file.iloc[row] #load the row (the cone)
            mapCone = Map.Cone(row, [coneToCovert['Cone_X'], coneToCovert['Cone_Y']], (coneToCovert['Cone_Type'] == 'RIGHT'), coneToCovert['Finish'])
            listToAppend = (mapObj.right_cone_list if mapCone.LorR else mapObj.left_cone_list)
            listToAppend.append(mapCone)
            if(coneToCovert['Finish']):
                mapObj.finish_line_cones.append(mapCone)
        ## now that all the cones are imported, their connections can be established
        combinedConeList = mapObj.left_cone_list + mapObj.right_cone_list #this list is (or at least really really should) be the mapObj equivalent of the map_file, with the same order of items.
        for row in range(len(map_file)):  ## this work because: (combinedConeList[row].ID == row) is true
            connections = [map_file.iloc[row]['Conn_A'], map_file.iloc[row]['Conn_B']]
            for coneIndex in connections:
                if(not np.isnan(coneIndex)):
                    combinedConeList[row].connections.append(combinedConeList[GF.findIndexByClassAttr(combinedConeList, 'ID', int(coneIndex))])
        if(self.coneConnecterPresent):
            try:
                import coneConnecting as CC
                for cone in combinedConeList:
                    for connectedCone in cone.connections:
                        dist, angle = GF.distAngleBetwPos(cone.position, connectedCone.position)
                        cone.coneConData.append(CC.coneConnection(angle, dist, -1.0))
                        #connectedCone.coneConData.append(CC.coneConnection(GF.radInv(angle), dist, -1.0)) #don't do this, becuase you'll do it double
            except Exception as excep:
                print("couldn't rebuild coneConData for connected cones, exception:", excep)
        return(mapObj)
    
    def generateFilename(self):
        """generate a unique filename using self.defaultFilename and datetime"""
        return(self.defaultFilename + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S") + self.fileExt)
    
    def save_map(self, mapToSave: Map, filename=None):
        """save map to excel file
            if no filename is provided, one with be automatically generated using self.generateFilename()"""
        if(filename is None): #automatic file naming
            filename = self.generateFilename()
        elif(not filename.endswith(self.fileExt)):
            filename += self.fileExt
        print("saving to file:", filename)
        #print(map_file)
        map_file = self.mapObjectToFile(mapToSave)
        map_file.to_excel(filename)
        return(filename)
    
    def load_map_file(self, map_file: pd.core.frame.DataFrame, whereToLoad=None):
        #print(map_file)
        returnMap = self.mapFileToObject(map_file)
        if(whereToLoad is not None):
            #copyImportMap(whereToLoad, returnMap) #has problems with car objects and clocks and all that nastyness
            whereToLoad.left_cone_list = returnMap.left_cone_list
            whereToLoad.right_cone_list = returnMap.right_cone_list
            whereToLoad.finish_line_cones = returnMap.finish_line_cones
            #whereToLoad.clockSet(whereToLoad.clock) #a terrible hack that can be removed once the clock system is reworked
            try:
                if(self.pathPlanningPresent):
                    whereToLoad.makeBoundrySplines()
            except Exception as excep:
                print("couldn't makeBoundrySplines after loading map into", whereToLoad, ", exception:", excep)
        return(returnMap)
    
    
    def laod_map(self, filename: str, whereToLoad=None):
        """load an excel (pandas) file and return/import it
            provide filename, fileExt will be added if it's not present"""
        if(not filename.endswith(self.fileExt)):
            filename += self.fileExt
        current_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = filename #init var
        if(not filename.startswith(current_dir)):
             map_path = os.path.join(current_dir, filename)
        print("loading map file:", map_path)
        map_file = pd.read_excel(map_path, index_col=0) #'index_col=0' lets the parser know that the indices are in the first column (and so the value in header row is unimportant)
        return(self.load_map_file(map_file, whereToLoad)) #this is just to avoid having the same code twice
    


# if __name__ == '__main__':
#     loader = mapLoader()
#     aMap = Map()
#     aMap.left_cone_list.append(Map.Cone(1, [0.25, 0.75], False, False))
#     aMap.left_cone_list.append(Map.Cone(3, [3.0, 1.25], False, True))
#     aMap.left_cone_list.append(Map.Cone(4, [5.75, 1.75], False, False)); aMap.left_cone_list[-1].connections.append(aMap.left_cone_list[-2]);  aMap.left_cone_list[-2].connections.append(aMap.left_cone_list[-1])
#     aMap.left_cone_list.append(Map.Cone(8, [8.5, 2.25], False, False)); aMap.left_cone_list[-1].connections.append(aMap.left_cone_list[-2]);  aMap.left_cone_list[-2].connections.append(aMap.left_cone_list[-1])
#     aMap.right_cone_list.append(Map.Cone(2, [0, 0.123456789], True, True))
#     aMap.right_cone_list.append(Map.Cone(5, [1, 0.123456789], True, False)); aMap.right_cone_list[-1].connections.append(aMap.right_cone_list[-2]);  aMap.right_cone_list[-2].connections.append(aMap.right_cone_list[-1])
#     aMap.right_cone_list.append(Map.Cone(6, [2, 0.123456789], True, False)); aMap.right_cone_list[-1].connections.append(aMap.right_cone_list[-2]);  aMap.right_cone_list[-2].connections.append(aMap.right_cone_list[-1])
#     aMap.right_cone_list.append(Map.Cone(7, [3, 0.123456789], True, False))
#     loader.save_map(aMap, "test")
#     returnedMap = loader.laod_map("test")
#     loader.save_map(returnedMap, "reTest")
