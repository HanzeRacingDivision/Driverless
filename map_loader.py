import pandas as pd
import numpy as np
import os
import datetime     #used for naming files automatically

from Map import Map
import GF.generalFunctions as GF

import time #for debugging


########################### some (static!!!) functions ####################################

def mapObjectToFile(mapToSave):
    """convert Map object to pandas dataframe"""
    combinedConeList = mapToSave.left_cone_list + mapToSave.right_cone_list
    map_file = pd.DataFrame({'ConeID' : [cone.ID for cone in combinedConeList],
                             'Cone_Type' : [('RIGHT' if cone.LorR else 'LEFT') for cone in combinedConeList],
                             'Cone_X' : [cone.position[0] for cone in combinedConeList],
                             'Cone_Y' : [cone.position[1] for cone in combinedConeList],
                             'Finish' : [cone.isFinish for cone in combinedConeList],
                             'Conn_A' : [(cone.connections[0].ID if (len(cone.connections)>0) else None) for cone in combinedConeList],
                             'Conn_B' : [(cone.connections[1].ID if (len(cone.connections)>1) else None) for cone in combinedConeList]})
    return(map_file)

def mapFileToObject(map_file):
    """convert pandas dataframe (map_file) to Map object"""
    mapObj = Map()
    if("ConeID" in map_file.columns): #storing the ConeID in the mapfile makes manual mapfile editing a lot easier and maploading in general less error-prone
        connectionList = [] # a list where we temporarily store the coneID's of connected cones alongside a cone pointer. [conePointer, ID, ID]
        for row in range(len(map_file)):
            coneToCovert = map_file.iloc[row] #load the row (the cone)
            addSuccess, coneInList = mapObj.addConeObj(Map.Cone(int(coneToCovert['ConeID']), [coneToCovert['Cone_X'], coneToCovert['Cone_Y']], (coneToCovert['Cone_Type'] == 'RIGHT'), coneToCovert['Finish']), False) #manually add cone object to make sure that the ID matches the stored one
            if(addSuccess):
                connectionList.append([coneInList, coneToCovert['Conn_A'], coneToCovert['Conn_B']])
            else:
                print("maploader failed to add a cone:", coneToCovert['ConeID'], coneInList)
        ## now that all the cones are imported, their connections can be established
        for i in range(len(connectionList)): # for every cone that has been loaded in
            LorRconeList = (mapObj.right_cone_list if connectionList[i][0].LorR else mapObj.left_cone_list) # alternatively, you could use a combinedConeList
            for connectedConeID in connectionList[i][1::]:
                if(not np.isnan(connectedConeID)):
                    connectionList[i][0].connections.append(LorRconeList[GF.findIndexByClassAttr(LorRconeList, 'ID', int(connectedConeID))])
    else: #old system (when ConeID was not stored in mapfile yet)
        print("importing mapfile using old system (no ConeID column)")
        for row in range(len(map_file)):
            coneToCovert = map_file.iloc[row] #load the row (the cone)
            mapObj.addConeObj(Map.Cone(row, [coneToCovert['Cone_X'], coneToCovert['Cone_Y']], (coneToCovert['Cone_Type'] == 'RIGHT'), coneToCovert['Finish'])) #manually add cone object to make sure that the ID matched the row index
        ## now that all the cones are imported, their connections can be established
        combinedConeList = mapObj.left_cone_list + mapObj.right_cone_list #this list is (or at least really really should) be the mapObj equivalent of the map_file, with the same order of items.
        for row in range(len(map_file)):  ## this works because: (combinedConeList[row].ID == row) is true
            connections = [map_file.iloc[row]['Conn_A'], map_file.iloc[row]['Conn_B']]
            for coneIndex in connections:
                if(not np.isnan(coneIndex)):
                    combinedConeList[row].connections.append(combinedConeList[GF.findIndexByClassAttr(combinedConeList, 'ID', int(coneIndex))])
    
    try:
        import coneConnecting as CC
        combinedConeList = mapObj.left_cone_list + mapObj.right_cone_list
        for cone in combinedConeList:
            for connectedCone in cone.connections:
                dist, angle = GF.distAngleBetwPos(cone.position, connectedCone.position)
                cone.coneConData.append(CC.coneConnection(angle, dist, -1.0))
                #connectedCone.coneConData.append(CC.coneConnection(GF.radInv(angle), dist, -1.0)) #don't do this, becuase you'll do it double
    except Exception as excep:
        print("couldn't rebuild coneConData for connected cones, exception:", excep)
    return(mapObj)

def generateFilename():
    """generate a unique filename using mapLoader.defaultFilename and datetime"""
    return(mapLoader.defaultFilename + datetime.datetime.now().strftime("%Y-%m-%d_%H;%M;%S") + mapLoader.fileExt)

def save_map(mapToSave: Map, filename=None, saveSimVarsMap=True):
    """save map to excel file
        if no filename is provided, one with be automatically generated using generateFilename()"""
    if(filename is None): #automatic file naming
        filename = generateFilename()
    elif(not filename.endswith(mapLoader.fileExt)):
        filename += mapLoader.fileExt
    print("saving to file:", filename)
    saveStartTime = time.time()
    map_file = mapObjectToFile(mapToSave)
    print("save_map pandas conversion time:", round(time.time()-saveStartTime,2)); saveStartTime=time.time()
    map_file.to_excel(filename)
    print("save_map .to_excel time:", round(time.time()-saveStartTime,2))
    # map_file_simVar = None
    if(saveSimVarsMap and (((len(mapToSave.simVars.left_cone_list)+len(mapToSave.simVars.right_cone_list))>0) if (mapToSave.simVars is not None) else False)):
        simVarMapFilename = "simVar_" + filename # add something to the start (so i dont have to deal with inserting before the file extension)
        #save_map(mapToSave.simVars, simVarMapFilename, False) # recursively
        print("saving simVar map to file:", simVarMapFilename)
        saveStartTime = time.time()
        map_file_simVar = mapObjectToFile(mapToSave.simVars)
        print("save_map simVar pandas conversion time:", round(time.time()-saveStartTime,2)); saveStartTime=time.time()
        map_file_simVar.to_excel(simVarMapFilename)
        print("save_map simVar .to_excel time:", round(time.time()-saveStartTime,2))
    return(filename, map_file) #, map_file_simVar)   # i think returning the mapFile came from an old version where it was sent to a remote client, i'm just gonna leave it...

def load_map_file(map_file: pd.core.frame.DataFrame, whereToLoad=None):
    #print(map_file)
    returnMap = mapFileToObject(map_file)
    if(whereToLoad is not None):
        # if(whereToLoad.simVars is not None): # MOVED to outside the funcion (drawDriverless (and the main file (ARCv0.py) for cmdLine loading))
        #     print("loading mapfile into .simVars")
        #     whereToLoad = whereToLoad.simVars
        #copyImportMap(whereToLoad, returnMap) #has problems with car objects and clocks and all that nastyness
        whereToLoad.left_cone_list = returnMap.left_cone_list
        whereToLoad.right_cone_list = returnMap.right_cone_list
        whereToLoad.finish_line_cones = returnMap.finish_line_cones
        #whereToLoad.clockSet(whereToLoad.clock) #a terrible hack that can be removed once the clock system is reworked
        try:
            import pathPlanningTemp as PP
            PP.makeBoundrySplines(whereToLoad)
        except Exception as excep:
            doNothing = 0
            #print("couldn't makeBoundrySplines after loading map into", whereToLoad, ", exception:", excep)
    # try:
    #     if(returnMap.pathPlanningPresent):
    #         import pathPlanningTemp as PP
    #         PP.makeBoundrySplines(returnMap)
    # except Exception as excep:
    #     print("couldn't makeBoundrySplines after loading map into", returnMap, ", exception:", excep)
    return(returnMap)


def load_map(filename: str, whereToLoad=None):
    """load an excel (pandas) file and return/import it
        provide filename, fileExt will be added if it's not present"""
    if(not filename.endswith(mapLoader.fileExt)):
        filename += mapLoader.fileExt
    current_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = filename #init var
    if(not filename.startswith(current_dir)):
         map_path = os.path.join(current_dir, filename)
    print("loading map file:", map_path)
    map_file = pd.read_excel(map_path, index_col=0) #'index_col=0' lets the parser know that the indices are in the first column (and so the value in header row is unimportant)
    return(load_map_file(map_file, whereToLoad)) #this is just to avoid having the same code twice



class mapLoader:
    """a static class for saving/loading/importing pandas excel files
        (currently!) only saves cone states (positions, connections, etc)"""
    fileExt = ".xlsx"
    defaultFilename = "map_"
    
    #this is mostly to keep compatibility with my older versions (where the pathPlanner class is inherited into the map object). I can't recommend that, as the map object is often transmitted to other processes/PCs
    @staticmethod
    def mapObjectToFile(mapToSave):
        return(mapObjectToFile(mapToSave))
    
    @staticmethod
    def mapFileToObject(map_file):
        return(mapFileToObject(map_file))
    
    @staticmethod
    def generateFilename():
        return(generateFilename)
    
    @staticmethod
    def save_map(mapToSave: Map, filename=None):
        return(save_map(mapToSave, filename))
    
    @staticmethod
    def load_map_file(map_file: pd.core.frame.DataFrame, whereToLoad=None):
        return(load_map_file(map_file, whereToLoad))
    
    @staticmethod
    def load_map(filename: str, whereToLoad=None):
        return(load_map(filename, whereToLoad))

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
#     returnedMap = loader.load_map("test")
#     loader.save_map(returnedMap, "reTest")
