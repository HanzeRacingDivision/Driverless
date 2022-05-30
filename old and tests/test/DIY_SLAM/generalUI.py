# this is a seperate file becuase now i can avoid importing pygame. Otherwise, i'd put this in pygameUI.py

from Map import Map


def UIparser(mapToUse, instruction):
    """parse the received instruction (like place-cone or start-auto)
        this fulfills the funcion of drawDriverless, but with less(?) CPU/GPU drag"""
    if(mapToUse is None):
        print("(UIreceiver) objectWithMap is None, can't parse UI")
        return(False)
    print("parsing:", instruction[0])
    if(instruction[0] == 'PLACE'):
        if(len(instruction) < 3): #if the 'immediateConnect' argument was not provided (for whatever silly reason...) assume it's False
            instruction.append(False)
        if(type(instruction[1]) is Map.Cone):
            conePlaceSuccess, coneInList = mapToUse.addConeObj(instruction[1])
        elif(type(instruction[1]) is tuple):
            conePlaceSuccess, coneInList = mapToUse.addCone(*instruction[1])
        else:
            print("can't enact 'PLACE' instruction, instuction format wrong:", instruction[1], type(instruction[1]))
            return(False)
        if(not conePlaceSuccess):
            print("can't enact 'PLACE' instruction, it overlaps with existing cone:", coneInList.ID)
            return(False)
        if(instruction[2]):
            if(mapToUse.coneConnecterPresent):
                import coneConnecting as CC
                CC.connectCone(mapToUse, coneInList)
            else:
                print("couldn't connect newly instructed cone, because mapToUse.coneConnecterPresent == False")
        if(mapToUse.pathPlanningPresent):
            import pathPlanningTemp as PP
            PP.makeBoundrySplines(mapToUse)
    elif(instruction[0] == 'CONNEC'):
        if(mapToUse.coneConnecterPresent):
            import coneConnecting as CC
            if(type(instruction[1]) is Map.Cone):
                overlaps, coneToConnect = mapToUse.overlapConeCheck(instruction[1].position) #this is needed to retrieve the LOCAL cone object, not the transmitted one
                if(overlaps and (coneToConnect.ID == instruction[1].ID)):
                    CC.connectCone(mapToUse, coneToConnect)
                    if(mapToUse.pathPlanningPresent):
                        import pathPlanningTemp as PP
                        PP.makeBoundrySplines(mapToUse)
                else:
                    print("'CONNEC' instruction couldnt find cone at location:", instruction[1].position, "with ID:", instruction[1].ID)
                    return(False)
            elif(type(instruction[1]) is tuple):
                overlaps, coneToConnect = mapToUse.overlapConeCheck(instruction[1][1])
                if(overlaps and (coneToConnect.ID == instruction[1][0])):
                    CC.connectCone(mapToUse, coneToConnect)
                    if(mapToUse.pathPlanningPresent):
                        import pathPlanningTemp as PP
                        PP.makeBoundrySplines(mapToUse)
                else:
                    print("'CONNEC' instruction couldnt find cone at location:", instruction[1][1], "with ID:", instruction[1][0])
                    return(False)
            else:
                print("can't enact 'CONNEC' instruction, instuction format wrong:", instruction[1], type(instruction[1]))
                return(False)
        else:
            print("ignored 'CONNEC' instruction, because mapToUse.coneConnecterPresent == False")
    elif(instruction[0] == 'SETFIN'):
        overlaps, coneToSetFin = mapToUse.overlapConeCheck(instruction[1].position) #this is needed to retrieve the LOCAL cone object, not the transmitted one
        if(overlaps and (coneToSetFin.ID == instruction[1].ID)):
            if((len(mapToUse.finish_line_cones)<2) and ((mapToUse.finish_line_cones[0].LorR != coneToSetFin.LorR) if (len(mapToUse.finish_line_cones)>0) else True)):
                coneToSetFin.isFinish = True
                mapToUse.finish_line_cones.append(coneToSetFin)
            else:
                print("couldn't enact 'SETFIN' instruction, too many finish cones OR", ("right" if coneToSetFin.LorR else "left"),"cone finish already exists")
        else:
            print("'SETFIN' instruction couldnt find cone at location:", instruction[1].position, "with ID:", instruction[1].ID)
            return(False)
    elif(instruction[0] == 'DELET'):
        if(type(instruction[1]) is Map.Cone):
            overlaps, coneToDelete = mapToUse.overlapConeCheck(instruction[1].position) #this is needed to retrieve the LOCAL cone object, not the transmitted one
            if(overlaps and (coneToDelete.ID == instruction[1].ID)):
                mapToUse.removeConeObj(coneToDelete)
                if(mapToUse.pathPlanningPresent):
                    import pathPlanningTemp as PP
                    PP.makeBoundrySplines(mapToUse)
            else:
                print("'DELET' instruction couldn't find cone at location:", instruction[1].position, "with ID:", instruction[1].ID)
                return(False)
        elif(type(instruction[1]) is tuple):
            overlaps, coneToDelete = mapToUse.overlapConeCheck(instruction[1][1])
            if(overlaps and (coneToDelete.ID == instruction[1][0])):
                mapToUse.removeConeObj(coneToDelete)
                if(mapToUse.pathPlanningPresent):
                    import pathPlanningTemp as PP
                    PP.makeBoundrySplines(mapToUse)
            else:
                print("'DELET' instruction couldn't find cone at location:", instruction[1][1], "with ID:", instruction[1][0])
                return(False)
        else:
            print("can't enact 'DELET' instruction, instuction format wrong:", instruction[1], type(instruction[1]))
            return(False)
    elif(instruction[0] == 'PATH'):
        if(mapToUse.pathFinderPresent):
            import pathFinding    as PF
            if(instruction[1] < 1): #if instructed to path-find as far as possible
                limitCounter = 0
                while(PF.makePath(mapToUse) and (limitCounter<25)): #stops when path can no longer be advanced
                    limitCounter += 1
            else: #if instructed to (attempt to) find a certian number of path points
                for i in range(instruction[1]): #try to make this many path points
                    PF.makePath(mapToUse)
            if(mapToUse.pathPlanningPresent):
                import pathPlanningTemp as PP
                PP.makePathSpline(mapToUse)
        else:
            print("ignored 'PATH' instruction, because mapToUse.pathFinderPresent == False")
    elif(instruction[0] == 'AUTO'):
        if(len(instruction) != 3):
            print("invalid 'AUTO' instruction, too few arguments")
            return(False)
        if(mapToUse.pathPlanningPresent):
            if(mapToUse.car.pathFolData is not None):
                mapToUse.car.pathFolData.auto = instruction[1]
                mapToUse.car.pathFolData.targetVelocity = instruction[2]
                if(not mapToUse.car.pathFolData.auto):
                    mapToUse.car.desired_velocity = 0.0 #mapToUse.car.pathFolData.targetVelocity
                    # try:
                    #     mapToUse.car.sendSpeedAngle(0.0, 0.0)
                    # except:
                    #     print("couldn't send stop command to car (sendSpeedAngle) after disabling auto")
            else:
                print("failed to perform 'AUTO' instruction, as car.pathFolData is None")
        else:
            print("ignored 'AUTO' instruction, because mapToUse.pathPlanningPresent == False")
    elif(instruction[0] == 'MAPLOD'):
        try:
            import map_loader as ML
            if(len(instruction) == 2):
                ML.load_map(instruction[1], mapToUse)
                print("loaded file successfully")
            elif(len(instruction) == 3):
                with open(instruction[1], 'wb') as writeFile:
                    writeFile.write(instruction[2]) #the excel file was sent over as as raw bytes, so save it like that too.
                ML.load_map(instruction[1], mapToUse)
                print("loaded file successfully")
            else:
                print("can't enact 'MAPLOD' instruction, instuction format wrong:", instruction)
            return(False)
        except Exception as excep:
            print("failed to perform 'MAPLOD' instruction, exception:", excep)
    elif(instruction[0] == 'MAPSAV'):
        try:
            import map_loader as ML
            import time
            saveStartTime = time.time()
            filename, map_file = ML.save_map(mapToUse, instruction[1]) #if instruction[1] (filename) is None, a filename will be autogenerated
            ## i noticed that save_map can get very very slow
            print("(remotely instructed) saving took", round(time.time()-saveStartTime, 2), "seconds")
        except Exception as excep:
            print("failed to perform 'MAPSAV' instruction, exception:", excep)
    else:
        print("can't parse unknown instruction:", instruction[0])
        return(False)
    return(True)