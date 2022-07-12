import pygame       #python game library, used for the visualization
import time         #used for debugging
import numpy as np  #general math library

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



def handleMousePress(pygameDrawerInput, buttonDown, button, pos, eventToHandle):
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
        pygame.event.set_grab(0) # allow the mouse to exit the window once again
        pygameDrawerInput.mouseCone = None # stop drawing the hovering preview
        posToPlace = pygameDrawerInput.pixelsToRealPos(pos) # convert to real units
        
        if(pygameDrawerInput.extraViewMode):
            print("ignoring mouse click because of extraViewMode!")
            return

        normalMap = pygameDrawerInput.mapToDraw # just a more legible pointer
        overlaps, overlappingCone = normalMap.overlapConeCheck(posToPlace)
        allowFinish = (normalMap.find_finish_cones(overlappingCone.LorR if overlaps else leftOrRight) is None)
        allowConnect = ((len(overlappingCone.connections) < 2) if overlaps else True)

        simMap = pygameDrawerInput.mapToDraw.simVars # either a map object or None
        overlapsUndis, overlappingUndisCone = (simMap.overlapConeCheck(posToPlace) if (simMap is not None) else (False, None))
        allowUndisFinish = ((simMap.find_finish_cones(overlappingUndisCone.LorR if overlapsUndis else leftOrRight) is None) if (simMap is not None) else False)
        allowUndisConnect = ((len(overlappingUndisCone.connections) < 2) if overlapsUndis else True)
        prioritizeSimMap = (pygameDrawerInput.mapToDraw.simVars.undiscoveredCones if (simMap is not None) else False) # if 'undiscoveredCones' == True, then prioritize the simMap for certain things
        
        clickRequestRemove = pygame.key.get_pressed()[pygame.K_r]
        clickRequestFinish = pygame.key.get_pressed()[pygame.K_f]
        clickRequestConnect = pygame.key.get_pressed()[pygame.K_LSHIFT]

        ## notes: it's pretty good, but connecting an undiscovered cone when there is a discovered cone on top of it is not possible 
        ##  (i'd love to turn off the lidar temporarily, or even better; pause the whole damn simulation...)
        ## same problem (presumably) for setting finish cones

        ## mouse click UI logic for placing, removing, connecting and setting (cones) as finish:
        if(clickRequestRemove): # if removing a cone
            if(overlaps):
                deleting = True
                for target in pygameDrawerInput.mapToDraw.target_list: #look through the entire list of targets (you could also just look at self.mapToDraw.target_list[-2])
                    if((overlappingCone.ID == target.coneConData.cones[overlappingCone.LorR].ID) if (target.coneConData is not None) else False): #if the only connected cone is ALSO (already) in the target_list, then you cant make any more path
                        print("can't delete cone, it's in pathList") # note: this is temporary, will be possible at some point (when i stop being lazy)
                        deleting = False
                if(deleting):
                    print("deleting cone:", overlappingCone.ID)
                    normalMap.removeConeObj(overlappingCone)
            if(((not overlaps) or prioritizeSimMap) and overlapsUndis):
                print("removing simVars cone")
                simMap.removeConeObj(overlappingUndisCone)
                ## loaded map was altered
        else: # if placing, connecting or setting (cone) as finish
            if(overlapsUndis and prioritizeSimMap): # if there's no normal cone, but there IS an undiscovered cone AND you explicitely want to interact with the simMap
                if(clickRequestFinish):
                    if(allowUndisFinish):
                        print("setting simVars cone as finish:", overlappingUndisCone.ID)
                        overlappingUndisCone.isFinish = True # as this is a pointer (not a copy) of the cone, this should affect the cone in the list
                    else:
                        print("could not set simVars finish cone!")
                # if(clickRequestConnect): # clickRequestConnect is implicit from clicking on an existing cone, but i might use prioritizeSimMap to require it or something (idk)
                else:
                    if(allowUndisConnect):
                        print("connecting simVars cone:", overlappingUndisCone.ID)
                        import coneConnecting as CC
                        connectSuccess, winningCone = CC.connectCone(simMap, overlappingUndisCone)
                    else:
                        print("could not simVars connect cone!")
            elif(overlaps):
                print("overlaps")
                if(clickRequestFinish):
                    if(allowFinish):
                        print("setting cone as finish:", overlappingCone.ID)
                        overlappingCone.isFinish = True
                    else:
                        print("could not set finish cone!")
                # if(clickRequestConnect): # clickRequestConnect is implicit from clicking on an existing cone, but i might use prioritizeSimMap to require it or something (idk)
                else:
                    if(allowConnect):
                        print("connecting cone:", overlappingCone.ID)
                        import coneConnecting as CC
                        connectSuccess, winningCone = CC.connectCone(normalMap, overlappingCone)
                    else:
                        print("could not connect cone!")
            else: # (there no normal cones or undiscovered cones where you clicked:) placing a new cone
                if(prioritizeSimMap):
                    conePlaceSuccess, coneInList = simMap.addCone(posToPlace, leftOrRight, clickRequestFinish) # addCone has its own 'allowFinish' checking
                    if(clickRequestConnect and conePlaceSuccess):
                        if(allowUndisConnect):
                            print("connecting newly placed simVars cone:", coneInList.ID)
                            import coneConnecting as CC
                            connectSuccess, winningCone = CC.connectCone(simMap, coneInList)
                        else:
                            print("could not connect newly placed cone!")
                else:
                    conePlaceSuccess, coneInList = normalMap.addCone(posToPlace, leftOrRight, clickRequestFinish) # addCone has its own 'allowFinish' checking
                    if(clickRequestConnect and conePlaceSuccess):
                        if(allowConnect):
                            print("connecting newly placed cone:", coneInList.ID)
                            import coneConnecting as CC
                            connectSuccess, winningCone = CC.connectCone(normalMap, coneInList)
                        else:
                            print("could not connect newply placed cone!")

        try:
            import pathPlanningTemp as PP
            PP.makeBoundrySplines(normalMap)
        except:
            doNothing = 0
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

def handleKeyPress(pygameDrawerInput, keyDown, key, eventToHandle):
    """(UI element) handle the key-press-events
        see README.MD (github file) for what the keys do (or just scroll down and decypher)"""
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
            import pathFinding    as PF
            #PF.makePath(pygameDrawerInput.mapToDraw) #find a single path point
            limitCounter = 0
            while(PF.makePath(pygameDrawerInput.mapToDraw) and (limitCounter<25)): #stops when path can no longer be advanced
                limitCounter += 1
            import pathPlanningTemp as PP
            PP.makePathSpline(pygameDrawerInput.mapToDraw)
        elif((key==pygame.K_a) or ((key==pygame.K_PLUS) or (key==pygame.K_EQUALS)) or (key==pygame.K_MINUS)):
            if(key==pygame.K_a): # a
                if(pygameDrawerInput.mapToDraw.car.pathFolData is not None):
                    pygameDrawerInput.mapToDraw.car.pathFolData.auto = not pygameDrawerInput.mapToDraw.car.pathFolData.auto
                    #if(not pygameDrawerInput.isRemote):
                    pygameDrawerInput.mapToDraw.car.desired_velocity = 0.0
                    pygameDrawerInput.mapToDraw.car.desired_steering = 0.0
                    try:
                        pygameDrawerInput.mapToDraw.car.sendSpeedAngle(pygameDrawerInput.mapToDraw.car.desired_velocity, pygameDrawerInput.mapToDraw.car.desired_steering)
                    except:
                        print("couldn't send stopping insctruction")
            elif((key==pygame.K_PLUS) or (key==pygame.K_EQUALS)): # +
                if(pygameDrawerInput.mapToDraw.car.pathFolData is not None):
                    pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity += 0.25
            elif(key==pygame.K_MINUS): # -
                if(pygameDrawerInput.mapToDraw.car.pathFolData is not None):
                    pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity -= 0.25
                    if(pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity < 0):
                        pygameDrawerInput.mapToDraw.car.pathFolData.targetVelocity = 0
        elif(key==pygame.K_q): # q (cubic splines sounds like 'q'-bic, also i suck at spelling)
            pygameDrawerInput.drawCubicSplines = not pygameDrawerInput.drawCubicSplines #only has (the desired) effect if pyagmesimInput.mapToDraw.pathPlanningPresent == True
        elif(key==pygame.K_t): # t
            pygameDrawerInput.drawTargetConeLines = not pygameDrawerInput.drawTargetConeLines #only has an effect if pyagmesimInput.mapToDraw.pathFinderPresent == True
        elif(key==pygame.K_l): # l
            pygameDrawerInput.drawConeSlamData += 1 #only has an effect if there is a lidar present
            if(pygameDrawerInput.drawConeSlamData > 3): #rollover
                pygameDrawerInput.drawConeSlamData = 0
        elif(key==pygame.K_h): # h
            if(pygameDrawerInput.carPolygonMode):
                pygameDrawerInput.carPolygonMode = False
            else:
                if(pygameDrawerInput.headlights):
                    pygameDrawerInput.carPolygonMode = True
                    pygameDrawerInput.headlights = False
                else:
                    pygameDrawerInput.headlights = True # only has an effect if car sprite is used (.carPolygonMode)
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
        # elif(key==pygame.K_RIGHTBRACKET):
        #     if(pygameDrawerInput.isRemote):
        #         pygameDrawerInput.remoteFPS += 2
        # elif(key==pygame.K_LEFTBRACKET):
        #     if(pygameDrawerInput.isRemote):
        #         pygameDrawerInput.remoteFPS -= 2
        #         if(pygameDrawerInput.remoteFPS <= 0):
        #             pygameDrawerInput.remoteFPS = 1
        elif(key==pygame.K_s): # s
            try:
                import map_loader as ML
                saveStartTime = time.time()
                mapfilename, _ = ML.save_map(pygameDrawerInput.mapToDraw) #currently, file name is auto-generated, because requesting UI text field input is a lot of effort, and python input() is blocking
                pygameDrawerInput.lastMapFilename = mapfilename
                print("save_map took", round(time.time()-saveStartTime, 2), "seconds")
            except Exception as excep:
                print("failed to save file, exception:", excep)
        elif(key==pygame.K_m): # m
            if(pygameDrawerInput.mapToDraw.simVars is not None):
                pygameDrawerInput.mapToDraw.simVars.undiscoveredCones = not pygameDrawerInput.mapToDraw.simVars.undiscoveredCones # toggle
        elif(key==pygame.K_z): # z
            pygameDrawerInput.centerZooming = not pygameDrawerInput.centerZooming # toggle
#        elif(key==pygame.K_d): # d   (for debug)
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

def handleWindowEvent(pygameDrawerInput, eventToHandle):
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
        try:
                #note: drag and drop functionality is a little iffy for multisim applications
            import map_loader as ML
            if((pygameDrawerInput.mapToDraw.simVars.undiscoveredCones) if (pygameDrawerInput.mapToDraw.simVars is not None) else False):
                print("loading mapfile into .simVars!")
                ML.load_map(eventToHandle.file, pygameDrawerInput.mapToDraw.simVars)
            else:
                ML.load_map(eventToHandle.file, pygameDrawerInput.mapToDraw)
            import os
            _, pygameDrawerInput.lastMapFilename = os.path.split(eventToHandle.file) # save the name of the loaded mapfile
            print("loaded file successfully")
        except Exception as excep:
            print("failed to load drag-dropped file, exception:", excep)
    
    elif((eventToHandle.type == pygame.MOUSEBUTTONDOWN) or (eventToHandle.type == pygame.MOUSEBUTTONUP)):
        #print("mouse press", eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos)
        #handleMousePress(currentpygameDrawerInput(pygameDrawerInputList, eventToHandle.pos, True), eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        handleMousePress(pygameDrawerInput, eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        
    elif((eventToHandle.type == pygame.KEYDOWN) or (eventToHandle.type == pygame.KEYUP)):
        #print("keypress:", eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key))
        #handleKeyPress(currentpygameDrawerInput(pygameDrawerInputList, None, True), eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, eventToHandle)
        handleKeyPress(pygameDrawerInput, eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, eventToHandle)
    
    elif(eventToHandle.type == pygame.MOUSEWHEEL): #scroll wheel (zooming / rotating)
        #simToScale = currentpygameDrawerInput(pygameDrawerInputList, None, True)
        simToScale = pygameDrawerInput
        if(pygame.key.get_pressed()[pygame.K_LCTRL] and simToScale.carCam): #if holding (left) CTRL while in carCam mode, rotate the view
            simToScale.carCamOrient += (eventToHandle.y * np.pi/16)
        else:
            # save some stuff before the change
            viewSizeBeforeChange = [simToScale.drawSize[0]/simToScale.sizeScale, simToScale.drawSize[1]/simToScale.sizeScale]
            mousePosBeforeChange = simToScale.pixelsToRealPos(pygame.mouse.get_pos())
            # update sizeScale
            simToScale.sizeScale *= 1.0+(eventToHandle.y/10.0) #10.0 is an arbetrary zoomspeed
            if(simToScale.sizeScale < simToScale.minSizeScale):
                print("can't zoom out any further")
                simToScale.sizeScale = simToScale.minSizeScale
            elif(simToScale.sizeScale > simToScale.maxSizeScale):
                simToScale.sizeScale = simToScale.maxSizeScale
            #if(not simToScale.carCam): #viewOffset is not used in carCam mode, but it won't hurt to change it anyway
            dif = None # init var
            if(simToScale.centerZooming): ## center zooming:
                dif = [(viewSizeBeforeChange[0]-(simToScale.drawSize[0]/simToScale.sizeScale))/2, (viewSizeBeforeChange[1]-(simToScale.drawSize[1]/simToScale.sizeScale))/2]
            else: ## mouse position based zooming:
                mousePosAfterChange = simToScale.pixelsToRealPos(pygame.mouse.get_pos())
                dif = [mousePosBeforeChange[0] - mousePosAfterChange[0], mousePosBeforeChange[1] - mousePosAfterChange[1]]
            simToScale.viewOffset[0] -= dif[0] #equalizes from the zoom to 'happen' from the middle of the screen
            simToScale.viewOffset[1] -= dif[1]


def handleAllWindowEvents(pygameDrawerInput):
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
            handleWindowEvent(pygameDrawerInput, eventToHandle)
    
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
