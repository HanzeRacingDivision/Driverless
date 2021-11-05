import pygame       #python game library, used for the visualization
import time         #used for debugging
import os #only used for loading the car sprite (os is used to get the filepath)\
import numpy as np  #general math library

from generalUI import UIparser   #useful for passthrough (if you're importing pygameUI, you dont need to also import generalUI)
import drawDriverless as DD
#note: coneConnecting, pathFinding and pathPlanningTemp are imported at runtime if/when needed

# global pygamesimInputLast
# pygamesimInputLast = None #to be filled

#cursor in the shape of a flag
flagCurs = ("ooo         ooooooooo   ",
            "oo ooooooooo...XXXX..ooo",
            "oo ....XXXX....XXXX....o",
            "oo ....XXXX....XXXX....o",
            "oo ....XXXX.XXX....XX..o",
            "oo XXXX....XXXX....XXXXo",
            "oo XXXX....XXXX....XXXXo",
            "oo XXXX....X...XXXX..XXo",
            "oo ....XXXX....XXXX....o",
            "oo ....XXXX....XXXX....o",
            "ooo....XXXX.ooooooooo..o",
            "oo ooooooooo         ooo",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ",
            "oo                      ")
flagCurs16  =  ("oooooooooooooooo", #1
                "oo ...XXX...XXXo",
                "oo ...XXX...XXXo",
                "oo XXX...XXX...o", #4
                "oo XXX...XXX...o",
                "oo ...XXX...XXXo",
                "oo ...XXX...XXXo",
                "oo XXX...XXX...o", #8
                "oo XXX...XXX...o",
                "oooooooooooooooo",
                "oo              ",
                "oo              ", #12
                "oo              ",
                "oo              ",
                "oo              ",
                "oo              ") #16
global flagCurs24Data, flagCurs16Data, flagCursorSet, deleteCursorSet
flagCurs24Data = ((24,24),(0,23)) + pygame.cursors.compile(flagCurs, 'X', '.', 'o')
flagCurs16Data = ((16,16),(0,15)) + pygame.cursors.compile(flagCurs16, 'X', '.', 'o')
flagCursorSet = False
deleteCursorSet = False



def handleMousePress(pygameDrawerInput, phantomMap, buttonDown, button, pos, eventToHandle):
    """(UI element) handle the mouse-press-events"""
    if(buttonDown and ((button == 1) or (button == 3))): #left/rigth mouse button pressed (down)
        pygame.event.set_grab(1)
        leftOrRight = (True if (button == 3) else False)
        if(not pygame.key.get_pressed()[pygame.K_r]):
            pygameDrawerInput.mouseCone = leftOrRight
        if(pygame.key.get_pressed()[pygame.K_f]):
            pygame.mouse.set_cursor(flagCurs16Data[0], flagCurs16Data[1], flagCurs16Data[2], flagCurs16Data[3]) #smaller flag cursor
    elif((button == 1) or (button == 3)): #if left/right mouse button released
        leftOrRight = (True if (button == 3) else False)
        pygame.event.set_grab(0)
        pygameDrawerInput.mouseCone = None
        if(pygame.key.get_pressed()[pygame.K_r]):
            overlaps, overlappingCone = pygameDrawerInput.mapToDraw.overlapConeCheck(pygameDrawerInput.pixelsToRealPos(pos))
            if(overlaps):
                deleting = True
                for target in pygameDrawerInput.mapToDraw.target_list: #look through the entire list of targets (you could also just look at self.mapToDraw.target_list[-2])
                    if((overlappingCone.ID == target.coneConData.cones[overlappingCone.LorR].ID) if (target.coneConData is not None) else False): #if the only connected cone is ALSO (already) in the target_list, then you cant make any more path
                        print("can't delete cone, it's in pathList") #just a lot easier, not impossible
                        deleting = False
                if(deleting):
                    print("deleting cone:", overlappingCone.ID)
                    if(phantomMap):
                        #phantomMap._custom_(('DELET', overlappingCone), 1)
                        phantomMap._custom_(('DELET', (overlappingCone.ID, overlappingCone.position)), 1)
                        # coneList = (pygameDrawerInput.mapToDraw.right_cone_list if overlappingCone.LorR else pygameDrawerInput.mapToDraw.left_cone_list)
                        # #coneListIndex = GF.findIndexByClassAttr(coneList, 'ID', overlappingCone.ID)
                        # coneListIndex = coneList.index(overlappingCone)
                        # phantomMap._logDelete_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),))
                    else:
                        pygameDrawerInput.mapToDraw.removeConeObj(overlappingCone)
            if(pygameDrawerInput.mapToDraw.pathPlanningPresent):
                import pathPlanningTemp as PP
                PP.makeBoundrySplines(pygameDrawerInput.mapToDraw)
                #TBD: phantomMap
        else:
            if((len(pygameDrawerInput.mapToDraw.finish_line_cones) < 2) if pygame.key.get_pressed()[pygame.K_f] else True):
                posToPlace = pygameDrawerInput.pixelsToRealPos(pos)
                overlaps, overlappingCone = pygameDrawerInput.mapToDraw.overlapConeCheck(posToPlace)
                if(overlaps):
                    if(pygame.key.get_pressed()[pygame.K_f]):
                        if((pygameDrawerInput.mapToDraw.finish_line_cones[0].LorR != overlappingCone.LorR) if (len(pygameDrawerInput.mapToDraw.finish_line_cones) > 0) else True):
                            overlappingCone.isFinish = True
                            pygameDrawerInput.mapToDraw.finish_line_cones.append(overlappingCone)
                            if(phantomMap):
                                phantomMap._custom_(('SETFIN', overlappingCone), 0)
                                # coneList = (pygameDrawerInput.mapToDraw.right_cone_list if overlappingCone.LorR else pygameDrawerInput.mapToDraw.left_cone_list)
                                # coneListIndex = coneList.index(overlappingCone)
                                # phantomMap._put_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),'isFinish'), True)
                                # phantomMap._put_((('finish_line_cones', len(pygameDrawerInput.mapToDraw.finish_line_cones)),), overlappingCone)
                        else:
                            print("can't set (existing) cone as finish, there's aready a "+("right" if leftOrRight else "left")+"-sided finish cone")
                    elif(pygameDrawerInput.mapToDraw.coneConnecterPresent or pygameDrawerInput.isRemote):
                        import coneConnecting as CC
                        connectSuccess, winningCone = CC.connectCone(pygameDrawerInput.mapToDraw, overlappingCone)
                        #if(connectSuccess): #this check is only to avoid needless overhead on the master thread. If the (exact same) function says the cone can't connect here, there's no reason to try over on the master
                        if(phantomMap):
                            ## the custom-instruction way (much cleaner, especially for pointers and such)
                            #phantomMap._custom_(('CONNEC', overlappingCone), 0)
                            phantomMap._custom_(('CONNEC', (overlappingCone.ID, overlappingCone.position)), 1)
                            ## the manual way
                            # coneList = (pygameDrawerInput.mapToDraw.right_cone_list if overlappingCone.LorR else pygameDrawerInput.mapToDraw.left_cone_list)
                            # #coneListIndex = GF.findIndexByClassAttr(coneList, 'ID', overlappingCone.ID)
                            # coneListIndex = coneList.index(overlappingCone)
                            # # # phantomMap._put_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),), overlappingCone) #overwrite existing cone entirely
                            # # phantomMap._put_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),('connections',len(overlappingCone.connections)-1)), overlappingCone.connections[-1]) #overwrite only connection data
                            # # phantomMap._put_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),('connections',len(overlappingCone.coneConData)-1)), overlappingCone.coneConData[-1]) #overwrite only connection data
                            # phantomMap._append_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),('connections',)), overlappingCone.connections) #overwrite only connection data
                            # phantomMap._append_((('right_cone_list' if overlappingCone.LorR else 'left_cone_list', coneListIndex),('coneConData',)), overlappingCone.coneConData) #overwrite only connection data
                            
                            # coneListIndex = coneList.index(winningCone)
                            # # # phantomMap._put_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),), winningCone) #overwrite existing cone entirely
                            # # phantomMap._put_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('connections',len(winningCone.connections)-1)), winningCone.connections[-1]) #overwrite only connection data
                            # # phantomMap._put_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('connections',len(winningCone.coneConData)-1)), winningCone.coneConData[-1]) #overwrite only connection data
                            # phantomMap._append_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('connections',)), winningCone.connections) #overwrite only connection data
                            # phantomMap._append_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('coneConData',)), winningCone.coneConData) #overwrite only connection data
                else:
                    # newConeID = GF.findMaxAttrIndex((pygameDrawerInput.mapToDraw.right_cone_list + pygameDrawerInput.mapToDraw.left_cone_list), 'ID')[1]
                    # aNewCone = Map.Cone(newConeID+1, posToPlace, leftOrRight, bool(pygame.key.get_pressed()[pygame.K_f]))
                    if(((pygameDrawerInput.mapToDraw.finish_line_cones[0].LorR != leftOrRight) if (len(pygameDrawerInput.mapToDraw.finish_line_cones) > 0) else True) if pygame.key.get_pressed()[pygame.K_f] else True):
                        conePlaceSuccess, coneInList = pygameDrawerInput.mapToDraw.addCone(posToPlace, leftOrRight, bool(pygame.key.get_pressed()[pygame.K_f]))
                        if(conePlaceSuccess):
                            connectSuccess=False; winningCone=None #init vars
                            if(pygame.key.get_pressed()[pygame.K_LSHIFT] and pygameDrawerInput.mapToDraw.coneConnecterPresent):
                                import coneConnecting as CC
                                connectSuccess, winningCone = CC.connectCone(pygameDrawerInput.mapToDraw, coneInList)
                            if(phantomMap):
                                ## the custom-instruction way
                                #phantomMap._custom_(('PLACE', coneInList, bool(pygame.key.get_pressed()[pygame.K_f])), 0)
                                phantomMap._custom_(('PLACE', (posToPlace, leftOrRight, bool(pygame.key.get_pressed()[pygame.K_f])), bool(pygame.key.get_pressed()[pygame.K_LSHIFT])), 0)
                                ## the manual way
                                # coneList = (pygameDrawerInput.mapToDraw.right_cone_list if coneInList.LorR else pygameDrawerInput.mapToDraw.left_cone_list)
                                # #coneListIndex = GF.findIndexByClassAttr(coneList, 'ID', coneInList.ID)
                                # coneListIndex = coneList.index(coneInList)
                                # phantomMap._put_((('right_cone_list' if coneInList.LorR else 'left_cone_list', coneListIndex),), coneInList)
                                # if(coneInList.isFinish):
                                #     # phantomMap._put_((('finish_line_cones', len(pygameDrawerInput.mapToDraw.finish_line_cones)-1),), coneInList)
                                #     phantomMap._append_((('finish_line_cones', ),), pygameDrawerInput.mapToDraw.finish_line_cones)
                                # if(connectSuccess):
                                #     coneListIndex = coneList.index(winningCone)
                                #     # # phantomMap._put_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),), winningCone) #overwrite existing cone entirely
                                #     # phantomMap._put_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('connections',len(winningCone.connections)-1)), winningCone.connections[-1]) #overwrite only connection data
                                #     # phantomMap._put_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('coneConData',len(winningCone.coneConData)-1)), winningCone.coneConData[-1]) #overwrite only connection data
                                #     phantomMap._append_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('connections', )), winningCone.connections) #overwrite only connection data
                                #     phantomMap._append_((('right_cone_list' if winningCone.LorR else 'left_cone_list', coneListIndex),('coneConData', )), winningCone.coneConData) #overwrite only connection data
                if(pygameDrawerInput.mapToDraw.pathPlanningPresent):
                    import pathPlanningTemp as PP
                    PP.makeBoundrySplines(pygameDrawerInput.mapToDraw)
                    #TBD: phantomMap?
        if(pygame.key.get_pressed()[pygame.K_f]): #flag cursor stuff
            pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3]) #smaller flag cursor
    elif(button==2): #middle mouse button
        if(buttonDown): #mouse pressed down
            if(not pygameDrawerInput.carCam):
                pygame.event.set_grab(1)
                pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
                pygameDrawerInput.movingViewOffset = True
                pygameDrawerInput.movingViewOffsetMouseStart = pygame.mouse.get_pos()
                pygameDrawerInput.prevViewOffset = (pygameDrawerInput.viewOffset[0], pygameDrawerInput.viewOffset[1])
        else:           #mouse released
            pygame.event.set_grab(0)
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
            pygameDrawerInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
            pygameDrawerInput.movingViewOffset = False

def handleKeyPress(pygameDrawerInput, phantomMap, keyDown, key, eventToHandle):
    """(UI element) handle the key-press-events"""
    if(key==pygame.K_f): # f
        global flagCursorSet
        if(keyDown):
            if(not flagCursorSet): #in pygame SDL2, holding a button makes it act like a keyboard button, and event gets spammed.
                pygame.event.set_grab(1)
                pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3])
                flagCursorSet = True
        else:
            pygame.event.set_grab(0)
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
            flagCursorSet = False
    elif(key==pygame.K_r): # r
        global deleteCursorSet
        if(keyDown):
            if(not deleteCursorSet): #in pygame SDL2, holding a button makes it act like a keyboard button, and event gets spammed.
                pygame.event.set_grab(1)
                pygame.mouse.set_cursor(*pygame.cursors.broken_x)
                deleteCursorSet = True
        else:
            pygame.event.set_grab(0)
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
            deleteCursorSet = False
    elif(keyDown): #most things only happen on keyDown, this just saves a few lines
        if(key==pygame.K_p): # p
            if(phantomMap):
                phantomMap._custom_(('PATH', -1), 0)
            elif(pygameDrawerInput.mapToDraw.pathFinderPresent):
                import pathFinding    as PF
                #PF.makePath(pygameDrawerInput.mapToDraw) #find a single path point
                limitCounter = 0
                while(PF.makePath(pygameDrawerInput.mapToDraw) and (limitCounter<25)): #stops when path can no longer be advanced
                    limitCounter += 1
                if(pygameDrawerInput.mapToDraw.pathPlanningPresent):
                    import pathPlanningTemp as PP
                    PP.makePathSpline(pygameDrawerInput.mapToDraw)
        elif((key==pygame.K_a) or ((key==pygame.K_PLUS) or (key==pygame.K_EQUALS)) or (key==pygame.K_MINUS)):
            if(key==pygame.K_a): # a
                if((pygameDrawerInput.mapToDraw.car.pathFolData is not None) and pygameDrawerInput.mapToDraw.pathPlanningPresent):
                    pygameDrawerInput.mapToDraw.car.pathFolData.auto = not pygameDrawerInput.mapToDraw.car.pathFolData.auto
                    if(not pygameDrawerInput.isRemote):
                        pygameDrawerInput.mapToDraw.car.desired_velocity = 0.0
                        pygameDrawerInput.mapToDraw.car.desired_steering = 0.0
                        try:
                            pygameDrawerInput.mapToDraw.car.sendSpeedAngle(pygameDrawerInput.mapToDraw.car.desired_velocity, pygameDrawerInput.mapToDraw.car.desired_steering)
                        except:
                            print("couldn't send stopping insctruction")
                    # if(phantomMap):
                    #     phantomMap._put_(('car','pathFolData','auto'), pygameDrawerInput.mapToDraw.car.pathFolData.auto)
                    #     phantomMap._put_(('car','desired_velocity'), 0.0)
                    #     phantomMap._put_(('car','desired_steering'), 0.0)
            elif((key==pygame.K_PLUS) or (key==pygame.K_EQUALS)): # +
                if((pygameDrawerInput.mapToDraw.car.pathFolData is not None) and pygameDrawerInput.mapToDraw.pathPlanningPresent):
                    pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity += 0.25
                    # if(phantomMap):
                    #     phantomMap._put_(('car','pathFolData','targetVelocity'), pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity)
            elif(key==pygame.K_MINUS): # -
                if((pygameDrawerInput.mapToDraw.car.pathFolData is not None) and pygameDrawerInput.mapToDraw.pathPlanningPresent):
                    pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity -= 0.25
                    if(pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity < 0):
                        pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity = 0
                    # if(phantomMap):
                    #     phantomMap._put_(('car','pathFolData','targetVelocity'), pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity)
            if(phantomMap):
                phantomMap._custom_(('AUTO', pygameDrawerInput.mapToDraw.car.pathFolData.auto, pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity), 0)
        elif(key==pygame.K_q): # q (cubic splines sounds like 'q'-bic, also i suck at spelling)
            pygameDrawerInput.drawQubicSplines = not pygameDrawerInput.drawQubicSplines #only has (the desired) effect if pyagmesimInput.mapToDraw.pathPlanningPresent == True
        elif(key==pygame.K_t): # t
            pygameDrawerInput.drawTargetConeLines = not pygameDrawerInput.drawTargetConeLines #only has an effect if pyagmesimInput.mapToDraw.pathFinderPresent == True
        elif(key==pygame.K_l): # l
            pygameDrawerInput.drawConeSlamData += 1 #only has an effect if there is a lidar present
            if(pygameDrawerInput.drawConeSlamData > 2): #rollover
                pygameDrawerInput.drawConeSlamData = 0
        elif(key==pygame.K_h): # h
            pygameDrawerInput.headlights = not pygameDrawerInput.headlights #only has an effect if car sprite is used (.carPolygonMode)
#        elif(key==pygame.K_d): # d
#            ## (debug) printing the pickle size of individual components in the map object
#            import pickle
#            print("total len:", len(pickle.dumps(pygameDrawerInput.mapToDraw)))
#            def getComponentPickleSize(obj, recursive=0, recursiveThresh=2000): #a function to (recursively)
#                subLengths = []
#                if(type(obj) in (list, tuple, np.ndarray)):
#                    for i in range(len(obj)):
#                        subLengths.append([i, len(pickle.dumps(obj[i]))])
#                        if(recursive and (subLengths[-1][1] > recursiveThresh)):
#                            subLengths[-1].append(getComponentPickleSize(obj[i], recursive-1, recursiveThresh))
#                            subLengths[-1].append(sum(item[1] for item in subLengths[-1][2]))
#                else:
#                    for attrName in dir(obj):
#                        if((not attrName.startswith('__')) and (not callable(getattr(obj, attrName)))): #if the attribute is not private (low level stuff) or a function (method)
#                            subLengths.append([attrName, len(pickle.dumps(getattr(obj, attrName)))])
#                            if(recursive and (subLengths[-1][1] > recursiveThresh)):
#                                subLengths[-1].append(getComponentPickleSize(getattr(obj, attrName), recursive-1, recursiveThresh))
#                subLengths.append(["total", sum(item[1] for item in subLengths)])
#                return(subLengths)
#            def printComponentPickleSizes(pickleSizeList, _recursion=0):
#                for item in pickleSizeList:
#                    print('\t'*_recursion, item[0:2])
#                    if(len(item)>2):
#                        printComponentPickleSizes(item[2], _recursion+1)
#            printComponentPickleSizes(getComponentPickleSize(pygameDrawerInput.mapToDraw, 2, 2000))
        elif(key==pygame.K_c): # c
            # if(pygame.key.get_pressed()[pygame.K_LCTRL]): #causes too many problems, just restart the program or run pygameDrawerInput.__init__ again
            #     #reset everything
            pygameDrawerInput.carHistPoints = []
            pygameDrawerInput.carHistTimer = pygameDrawerInput.mapToDraw.clock() #reset timer
            pygameDrawerInput.drawCarHist = not pygameDrawerInput.drawCarHist
        elif(key==pygame.K_v): # v
            if(pygame.key.get_pressed()[pygame.K_LCTRL]):
                pygameDrawerInput.extraViewMode = not pygameDrawerInput.extraViewMode
            else:
                pygameDrawerInput.carCam = not pygameDrawerInput.carCam
                if(pygameDrawerInput.carCam and pygameDrawerInput.movingViewOffset): #if you switched to carCam while you were moving viewOffset, just stop moving viewOffset (same as letting go of MMB)
                    pygame.event.set_grab(0)
                    pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
                    pygameDrawerInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
                    pygameDrawerInput.movingViewOffset = False
        elif(key==pygame.K_RIGHTBRACKET):
            if(pygameDrawerInput.isRemote):
                pygameDrawerInput.remoteFPS += 2
                phantomMap._custom_(('FPSADJ', int(pygameDrawerInput.remoteFPS)), 2)
        elif(key==pygame.K_LEFTBRACKET):
            if(pygameDrawerInput.isRemote):
                pygameDrawerInput.remoteFPS -= 2
                if(pygameDrawerInput.remoteFPS <= 0):
                    pygameDrawerInput.remoteFPS = 1
                phantomMap._custom_(('FPSADJ', int(pygameDrawerInput.remoteFPS)), 2)
        elif(key==pygame.K_s): # s
            if(pygameDrawerInput.isRemote):
                phantomMap._custom_(('MAPSAV', None), 2)
            else:
                try:
                    import map_loader as ML
                    saveStartTime = time.time()
                    ML.save_map(pygameDrawerInput.mapToDraw) #currently, file name is auto-generated, because requesting UI text field input is a lot of effort, and python input() is blocking
                    print("save_map took", round(time.time()-saveStartTime, 2), "seconds")
                except Exception as excep:
                    print("failed to save file, exception:", excep)
# def currentpygameDrawerInput(pygameDrawerInputList, mousePos=None, demandMouseFocus=True): #if no pos is specified, retrieve it using get_pos()
#     """(UI element) return the pygameDrawer that the mouse is hovering over, or the one you interacted with last"""
#     if(len(pygameDrawerInputList) > 1):
#         if(mousePos is None):
#             mousePos = pygame.mouse.get_pos()
#         global pygameDrawerInputLast
#         if(pygame.mouse.get_focused() or (not demandMouseFocus)):
#             for pygameDrawerInput in pygameDrawerInputList:
#                 # localBoundries = [[pygameDrawerInput.drawOffset[0], pygameDrawerInput.drawOffset[1]], [pygameDrawerInput.drawSize[0], pygameDrawerInput.drawSize[1]]]
#                 # if(((mousePos[0]>=localBoundries[0][0]) and (mousePos[0]<(localBoundries[0][0]+localBoundries[1][0]))) and ((mousePos[1]>=localBoundries[0][1]) and (mousePos[0]<(localBoundries[0][1]+localBoundries[1][1])))):
#                 if(pygameDrawerInput.isInsideWindowPixels(mousePos)):
#                     pygameDrawerInputLast = pygameDrawerInput
#                     return(pygameDrawerInput)
#         if(pygameDrawerInputLast is None): #if this is the first interaction
#             pygameDrawerInputLast = pygameDrawerInputList[0]
#         return(pygameDrawerInputLast)
#     else:
#         return(pygameDrawerInputList[0])

def handleWindowEvent(pygameDrawerInput, phantomMap, eventToHandle):
    """(UI element) handle general (pygame) window-event"""
    if(eventToHandle.type == pygame.QUIT):
        #global windowKeepRunning
        print("pygame.QUIT")
        DD.windowKeepRunning = False #stop program (soon)
    
    elif(eventToHandle.type == pygame.VIDEORESIZE):
        newSize = eventToHandle.size
        if((DD.oldWindowSize[0] != newSize[0]) or (DD.oldWindowSize[1] != newSize[1])): #if new size is actually different
            print("VIDEORESIZE from", DD.oldWindowSize, "to", newSize)
            correctedSize = [newSize[0], newSize[1]]
            DD.window = pygame.display.set_mode(correctedSize, pygame.RESIZABLE)
            #for pygameDrawerInput in pygameDrawerInputList:
            localNewSize = [int((pygameDrawerInput.drawSize[0]*correctedSize[0])/DD.oldWindowSize[0]), int((pygameDrawerInput.drawSize[1]*correctedSize[1])/DD.oldWindowSize[1])]
            localNewDrawPos = [int((pygameDrawerInput.drawOffset[0]*correctedSize[0])/DD.oldWindowSize[0]), int((pygameDrawerInput.drawOffset[1]*correctedSize[1])/DD.oldWindowSize[1])]
            pygameDrawerInput.updateWindowSize(localNewSize, localNewDrawPos, autoMatchSizeScale=False)
        DD.oldWindowSize = DD.window.get_size() #update size (get_size() returns tuple of (width, height))
    
    elif(eventToHandle.type == pygame.WINDOWSIZECHANGED): # pygame 2.0.1 compatible    (right now (aug 2021, pygame 2.0.1 (SDL 2.0.14, Python 3.8.3)) both get called (on windows at least), but it should be fine)
        newSize = DD.window.get_size()
        if((DD.oldWindowSize[0] != newSize[0]) or (DD.oldWindowSize[1] != newSize[1])): #if new size is actually different
            print("WINDOWSIZECHANGED from", DD.oldWindowSize, "to", newSize)
            correctedSize = [newSize[0], newSize[1]]
            #for pygameDrawerInput in pygameDrawerInputList:
            localNewSize = [int((pygameDrawerInput.drawSize[0]*correctedSize[0])/DD.oldWindowSize[0]), int((pygameDrawerInput.drawSize[1]*correctedSize[1])/DD.oldWindowSize[1])]
            localNewDrawPos = [int((pygameDrawerInput.drawOffset[0]*correctedSize[0])/DD.oldWindowSize[0]), int((pygameDrawerInput.drawOffset[1]*correctedSize[1])/DD.oldWindowSize[1])]
            pygameDrawerInput.updateWindowSize(localNewSize, localNewDrawPos, autoMatchSizeScale=False)
        DD.oldWindowSize = DD.window.get_size() #update size (get_size() returns tuple of (width, height))
    
    elif(eventToHandle.type == pygame.DROPFILE): #drag and drop files to import them
        #pygameDrawerInput = currentpygameDrawerInput(pygameDrawerInputList, None, False)
        print("attempting to load drag-dropped file:", eventToHandle.file)
        if(phantomMap):
            if(pygameDrawerInput.isRemote):
                try:
                    strippedFileName = os.path.basename(eventToHandle.file)
                    print("sending map_file to remote client:", strippedFileName)
                    phantomMap._custom_(('MAPLOD', strippedFileName, open(eventToHandle.file, "rb").read()), 2)
                except Exception as excep:
                    print("failed to read dropFile (to send to remote instance):", excep)
            else:
                print("loading map_file (on another thread):", os.path.basename(eventToHandle.file))
                phantomMap._custom_(('MAPLOD', eventToHandle.file), 2)
        else:
            try:
                import map_loader as ML
                ML.load_map(eventToHandle.file, pygameDrawerInput.mapToDraw) #note: drag and drop functionality is a little iffy for multisim applications
                print("loaded file successfully")
                #TBD: phantomMap
            except Exception as excep:
                print("failed to load drag-dropped file, exception:", excep)
    
    elif((eventToHandle.type == pygame.MOUSEBUTTONDOWN) or (eventToHandle.type == pygame.MOUSEBUTTONUP)):
        #print("mouse press", eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos)
        #handleMousePress(currentpygameDrawerInput(pygameDrawerInputList, eventToHandle.pos, True), eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        handleMousePress(pygameDrawerInput, phantomMap, eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        
    elif((eventToHandle.type == pygame.KEYDOWN) or (eventToHandle.type == pygame.KEYUP)):
        #print("keypress:", eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key))
        #handleKeyPress(currentpygameDrawerInput(pygameDrawerInputList, None, True), eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, eventToHandle)
        handleKeyPress(pygameDrawerInput, phantomMap, eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, eventToHandle)
    
    elif(eventToHandle.type == pygame.MOUSEWHEEL): #scroll wheel (zooming / rotating)
        #simToScale = currentpygameDrawerInput(pygameDrawerInputList, None, True)
        simToScale = pygameDrawerInput
        if(pygame.key.get_pressed()[pygame.K_LCTRL] and simToScale.carCam): #if holding (left) CTRL while in carCam mode, rotate the view
            simToScale.carCamOrient += (eventToHandle.y * np.pi/16)
        else:
            dif = [simToScale.drawSize[0]/simToScale.sizeScale, simToScale.drawSize[1]/simToScale.sizeScale]
            #simToScale.sizeScale += eventToHandle.y #zooming (note: can reach 0, at which point the porgram crashes)
            simToScale.sizeScale *= 1.0+(eventToHandle.y/10.0) #10.0 is an arbetrary zoomspeed
            if(simToScale.sizeScale < 1.0):
                print("can't zoom out any further")
                simToScale.sizeScale = 1.0
            #if(not simToScale.carCam): #viewOffset is not used in carCam mode, but it won't hurt to change it anyway
            dif[0] -= (simToScale.drawSize[0]/simToScale.sizeScale)
            dif[1] -= (simToScale.drawSize[1]/simToScale.sizeScale)
            simToScale.viewOffset[0] -= dif[0]/2 #equalizes from the zoom to 'happen' from the middle of the screen
            simToScale.viewOffset[1] -= dif[1]/2

def handleAllWindowEvents(pygameDrawerInput, phantomMap=None):
    """(UI element) loop through (pygame) window-events and handle all of them"""
    # pygameDrawerInputList = []
    # if(type(pygameDrawerInput) is list):
    #     if(len(pygameDrawerInput) > 0):
    #         for entry in pygameDrawerInput:
    #             if(type(entry) is list):
    #                 for subEntry in entry:
    #                     pygameDrawerInputList.append(subEntry) #2D lists
    #             else:
    #                 pygameDrawerInputList.append(entry) #1D lists
    # else:
    #     pygameDrawerInputList = [pygameDrawerInput] #convert to 1-sizes array
    # #pygameDrawerInputList = pygameDrawerInput #assume input is list of pygamesims
    # if(len(pygameDrawerInputList) < 1):
    #     print("len(pygameDrawerInputList) < 1")
    #     #global windowKeepRunning
    #     DD.windowKeepRunning = False
    #     pygame.event.pump()
    #     return()
    for eventToHandle in pygame.event.get(): #handle all events
        if(eventToHandle.type != pygame.MOUSEMOTION): #skip mousemotion events early (fast)
            #handleWindowEvent(pygameDrawerInputList, eventToHandle)
            handleWindowEvent(pygameDrawerInput, phantomMap, eventToHandle)
    
    # #the manual keyboard driving (tacked on here, because doing it with the event system would require more variables, and this is temporary anyway)
    # #simToDrive = currentpygameDrawerInput(pygameDrawerInputList, demandMouseFocus=False) #get the active sim within the window
    # simToDrive = pygameDrawerInput
    # carToDrive = simToDrive.mapToDraw.car
    # if((not carToDrive.pathFolData.auto) if simToDrive.mapToDraw.pathPlanningPresent else True):
    #     pressedKeyList = pygame.key.get_pressed()
    #     deltaTime = time.time() - simToDrive.carKeyboardControlTimer
    #     simToDrive.carKeyboardControlTimer = time.time()
    #     speedAccelVal = 3.0 * deltaTime
    #     steerAccelVal = 1.5 * deltaTime
    #     #first for speed
    #     if(pressedKeyList[pygame.K_UP]): #accelerate button
    #         carToDrive.velocity += speedAccelVal #accelerate
    #     elif(pressedKeyList[pygame.K_DOWN]): #brake/reverse button
    #         if(carToDrive.velocity > (speedAccelVal*3)): #positive speed
    #             carToDrive.velocity -= speedAccelVal * 2 #fast brake
    #         else:                               #near-zero or negative speed
    #             carToDrive.velocity -= speedAccelVal * 0.5 #reverse accelerate
    #     else:                           #neither buttons
    #         if(carToDrive.velocity > speedAccelVal): #positive speed
    #             carToDrive.velocity -= speedAccelVal/2 #slow brake
    #         elif(carToDrive.velocity < -speedAccelVal): #negative speed
    #             carToDrive.velocity += speedAccelVal #brake
    #         else:                           #near-zero speed
    #             carToDrive.velocity = 0
    #     carToDrive.velocity = max(-1, min(2, carToDrive.velocity)) #limit speed
    #     #now for steering
    #     if(pressedKeyList[pygame.K_LEFT] and (not pressedKeyList[pygame.K_RIGHT])):
    #         carToDrive.steering += steerAccelVal
    #     elif(pressedKeyList[pygame.K_RIGHT] and (not pressedKeyList[pygame.K_LEFT])):
    #         carToDrive.steering -= steerAccelVal
    #     else:
    #         if(carToDrive.steering > steerAccelVal):
    #             carToDrive.steering -= steerAccelVal*2.0
    #         elif(carToDrive.steering < -steerAccelVal):
    #             carToDrive.steering += steerAccelVal*2.0
    #         else:
    #             carToDrive.steering = 0
    #     carToDrive.steering = max(np.deg2rad(-25), min(np.deg2rad(25), carToDrive.steering)) #limit speed
    #     # if(phantomMap):
    #     #     phantomMap._put_(('car','velocity'), carToDrive.velocity)
    #     #     phantomMap._put_(('car','steering'), carToDrive.steering)
