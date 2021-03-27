import pygame       #python game library, used for the visualization
import numpy as np  #general math library
import time         #used for (temporary) driving math in raceCar() class

from mapClassTemp import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use




class pygameDrawer():
    def __init__(self, mapToDraw, window, drawSize=(1200,600), drawOffset=(0,0), viewOffset=(0,0), carCamOrient=0, sizeScale=30, startWithCarCam=False, invertYaxis=True):
        self.window = window #pass on the window object (pygame)
        self.drawSize = (int(drawSize[0]),int(drawSize[1])) #width and height of the display area (does not have to be 100% of the window)
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1])) #draw position offset, (0,0) is topleft
        self.viewOffset = [float(viewOffset[0]), float(viewOffset[1])] #'camera' view offsets, changing this affects the real part of realToPixelPos()
        self.carCamOrient = carCamOrient #orientation of the car (and therefore everything) on the screen. 0 is towards the right
        self.sizeScale = sizeScale #pixels per meter
        self.carCam = startWithCarCam #it's either carCam (car-centered cam, with rotating but no viewOffset), or regular cam (with viewOffset, but no rotating)
        self.invertYaxis = invertYaxis #pygame has pixel(0,0) in the topleft, so this just flips the y-axis when drawing things
        
        self.mapToDraw = mapToDraw
        
        self.bgColor = [50,50,50] #grey
        
        self.finishLineColor = [255,40,0]
        self.finishLineWidth = 2 #pixels wide
        
        self.leftConeColor = [255,255,0] #yellow
        self.rightConeColor = [0,50,255] #dark blue
        self.coneLineWidth = 2 #pixels wide
        
        self.pathColor = [0,220,255] #light blue
        self.pathLineWidth = 2 #pixels wide
        #self.pathCenterPixelDiam = 
        
        self.mouseCone = None #either None, True or False, to indicate the color of the (mouse) cone that is about to be placed (replaces floatingCone)
        
        #the drawing stuff:
        self.carColor = [50,200,50]
        #polygon stuff (to be replaced by sprite?)
        self.carPointRadius = None #will be calculated once the car is drawn for the first time
        self.carPointAngle = None #will be calculated once the car is drawn for the first time
        
        self.movingViewOffset = False
        self.prevViewOffset = (self.viewOffset[0], self.viewOffset[1])
        self.movingViewOffsetMouseStart = [0,0]
        
        self.debugLines = [] #[[lineType, pos, pos/angles, color_index (0-2)], ] #third entry is: (lineType==0: straight line from two positions), (lineType==1: straight line from pos and [radius, angle]), (lineType==2: arc from pos and [radius, startAngle, endAngle])
        self.debugLineColors = [[255,0,255],[255,255,255],[0,255,0], [255,160,255]] #purple, white, green, pink
        self.debugLineWidth = 3
        
        ## load car sprite here (TBD)
        self.carPolygonMode = True #if no sprite is present, this can be used
        
        self.coneConnecterPresent = False
        self.pathFinderPresent = False
        self.pathPlanningPresent = False
        self.SLAMPresent = False
        
        #DELETE ME
        self.timeSinceLastUpdate = time.time()
        self.drawTargetConeLines = True #just for UI purposes, to toggle between showing and not showing how the targets are made
        self.carKeyboardControlTimer = time.time()
        self.carHistTimer = time.time()
        self.carHistPoints = []
        self.carHistTimeStep = 0.15
        self.carHistMinSquaredDistThresh = 0.1**2 #only save new positions if the car is moving
        self.carHistMaxLen = 200
        
        # try: #if there's no car object, this will not crash the entire program
        #     self.viewOffset = [(-self.car.pos[0]) + ((self.drawSize[0]/self.sizeScale)/2), (-self.car.pos[1]) + ((self.drawSize[1]/self.sizeScale)/2)]
        # except Exception as theExcept:
        #     print("couldn't set viewOffset to car pos:", theExcept)
    
    #pixel conversion functions (the most important functions in here)
    def pixelsToRealPos(self, pixelPos):
        if(self.carCam):
            dist = 0; angle = 0; #init var
            if(self.invertYaxis):
                dist, angle = GF.distAngleBetwPos([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2], [pixelPos[0], self.drawOffset[1]+(self.drawOffset[1]+self.drawSize[1])-pixelPos[1]]) #get distance to, and angle with respect to, center of the screen (car)
            else:
                dist, angle = GF.distAngleBetwPos([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2], pixelPos) #get distance to, and angle with respect to, center of the screen (car)
            return(GF.distAnglePosToPos(dist/self.sizeScale, angle+self.mapToDraw.car.angle-self.carCamOrient, self.mapToDraw.car.position)) #use converted dist, correctly offset angle & the real car pos to get a new real point
        else:
            if(self.invertYaxis):
                return([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((self.drawSize[1]-pixelPos[1]+self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]])
            else:
                return([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((pixelPos[1]-self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]])
    
    def realToPixelPos(self, realPos):
        if(self.carCam):
            dist, angle = GF.distAngleBetwPos(self.mapToDraw.car.position, realPos) #get distance to, and angle with respect to, car
            shiftedPixelPos = GF.distAnglePosToPos(dist*self.sizeScale, angle-self.mapToDraw.car.angle+self.carCamOrient, (self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2)) #calculate new (pixel) pos from the car pos, at the same distance, and the angle, plus the angle that the entire scene is shifted
            if(self.invertYaxis):
                return([shiftedPixelPos[0], self.drawOffset[1]+((self.drawOffset[1]+self.drawSize[1])-shiftedPixelPos[1])]) #invert Y-axis for normal (0,0) at bottomleft display
            else:
                return(shiftedPixelPos)
        else:
            if(self.invertYaxis):
                return([((realPos[0]+self.viewOffset[0])*self.sizeScale)+self.drawOffset[0], self.drawSize[1]-((realPos[1]+self.viewOffset[1])*self.sizeScale)+self.drawOffset[1]]) #invert Y-axis for normal (0,0) at bottomleft display
            else:
                return([((realPos[0]+self.viewOffset[0])*self.sizeScale)+self.drawOffset[0], ((realPos[1]+self.viewOffset[1])*self.sizeScale)+self.drawOffset[1]])
    
    #check if things need to be drawn at all
    def isInsideWindowPixels(self, pixelPos):
        return((pixelPos[0] < (self.drawSize[0] + self.drawOffset[0])) and (pixelPos[0] > self.drawOffset[0]) and (pixelPos[1] < (self.drawSize[1] + self.drawOffset[1])) and (pixelPos[1] > self.drawOffset[1]))
    
    def isInsideWindowReal(self, realPos):
        return(self.isInsideWindowPixels(self.realToPixelPos(realPos))) #not very efficient, but simple
    
    #drawing functions
    def background(self):
        self.window.fill(self.bgColor, (self.drawOffset[0], self.drawOffset[1], self.drawSize[0], self.drawSize[1])) #dont fill entire screen, just this pygamesim's area (allowing for multiple sims in one window)
    
    def drawCones(self, drawLines=True):
        conePixelDiam = Map.Cone.coneDiam * self.sizeScale
        drawnLineList = [] #[ [ID, ID], ] just a list of drawn lines by ID
        combinedConeList = (self.mapToDraw.right_cone_list + self.mapToDraw.left_cone_list)
        for cone in combinedConeList:
            #if(self.isInsideWindowReal(cone.position)): #if it is within bounds, draw it
            conePos = self.realToPixelPos(cone.position) #convert to pixel positions
            coneColor = self.rightConeColor if cone.LorR else self.leftConeColor
            if(drawLines):
                alreadyDrawn = [False, False]
                for drawnLine in drawnLineList:
                    for i in range(len(cone.connections)):
                        if((cone.ID in drawnLine) and (cone.connections[i].ID in drawnLine)):
                            alreadyDrawn[i] = True
                for i in range(len(cone.connections)):
                    if(not alreadyDrawn[i]):
                        pygame.draw.line(self.window, coneColor, conePos, self.realToPixelPos(cone.connections[i].position), self.coneLineWidth)
                        drawnLineList.append([cone.ID, cone.connections[i].ID]) #put established 'back' connection in list of drawn 
            #pygame.draw.circle(self.window, coneColor, [int(conePos[0]), int(conePos[1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
            conePos = GF.ASA(-(conePixelDiam/2), conePos) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
            pygame.draw.ellipse(self.window, coneColor, [conePos, [conePixelDiam, conePixelDiam]]) #draw cone
    
    def drawPathLines(self, drawConeLines=True, drawCenterPoints=False):
        # target_list content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        pathCenterPixelDiam = Map.Cone.coneDiam * self.sizeScale
        for i in range(len(self.mapToDraw.target_list)):
            #if(self.isInsideWindowReal(self.mapToDraw.target_list[i].position)):
            if(drawCenterPoints):
                #centerPixelPos = self.realToPixelPos(self.mapToDraw.target_list[i][0])
                #pygame.draw.circle(self.window, self.pathColor, [int(centerPixelPos[0]), int(centerPixelPos[1])], int(pathCenterPixelDiam/2)) #draw center point (as filled circle, not ellipse)
                pygame.draw.ellipse(self.window, self.pathColor, [GF.ASA(-(pathCenterPixelDiam/2), self.realToPixelPos(self.mapToDraw.target_list[i].position)), [pathCenterPixelDiam, pathCenterPixelDiam]]) #draw center point
            if(drawConeLines and self.pathFinderPresent):
                pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.mapToDraw.target_list[i].coneConData.cones[0].position), self.realToPixelPos(self.mapToDraw.target_list[i].coneConData.cones[1].position), self.pathLineWidth) #line from left cone to right cone
            if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                #draw line between center points of current pathline and previous pathline (to make a line that the car should (sort of) follow)
                pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.mapToDraw.target_list[i-1].position), self.realToPixelPos(self.mapToDraw.target_list[i].position), self.pathLineWidth) #line from center pos to center pos
        if(self.mapToDraw.targets_full_circle):
            pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.mapToDraw.target_list[-1].position), self.realToPixelPos(self.mapToDraw.target_list[0].position), self.pathLineWidth) #line that loops around to start
    
    def drawFinishLine(self):
        if(len(self.mapToDraw.finish_line_cones) >= 2):
            pygame.draw.line(self.window, self.finishLineColor, self.realToPixelPos(self.mapToDraw.finish_line_cones[0].position), self.realToPixelPos(self.mapToDraw.finish_line_cones[1].position), self.finishLineWidth)
    
    def drawCar(self):
        ## drawing is currently done by calculating the position of the corners and drawing a polygon with those points. Not efficient, not pretty, but fun
        #if(self.isInsideWindowReal(self.mapToDraw.car.position)):
        if(self.carPolygonMode):
            if(self.carPointRadius is None):
                self.carPointRadius = (((self.mapToDraw.car.width**2)+(self.mapToDraw.car.length**2))**0.5)/2 #Pythagoras
                self.carPointAngle = np.arctan2(self.mapToDraw.car.width, self.mapToDraw.car.length) #this is used to make corner point for polygon
            polygonPoints = []
            offsets = [[np.cos(self.carPointAngle+self.mapToDraw.car.angle) * self.carPointRadius, np.sin(self.carPointAngle+self.mapToDraw.car.angle) * self.carPointRadius],
                        [np.cos(np.pi-self.carPointAngle+self.mapToDraw.car.angle) * self.carPointRadius, np.sin(np.pi-self.carPointAngle+self.mapToDraw.car.angle) * self.carPointRadius]]
            polygonPoints.append(self.realToPixelPos([self.mapToDraw.car.position[0] + offsets[0][0], self.mapToDraw.car.position[1] + offsets[0][1]])) #front left
            polygonPoints.append(self.realToPixelPos([self.mapToDraw.car.position[0] + offsets[1][0], self.mapToDraw.car.position[1] + offsets[1][1]])) #back left
            polygonPoints.append(self.realToPixelPos([self.mapToDraw.car.position[0] - offsets[0][0], self.mapToDraw.car.position[1] - offsets[0][1]])) #back right
            polygonPoints.append(self.realToPixelPos([self.mapToDraw.car.position[0] - offsets[1][0], self.mapToDraw.car.position[1] - offsets[1][1]])) #front right
            pygame.draw.polygon(self.window, self.carColor, polygonPoints) #draw car
            #arrow drawing (not needed, just handy to indicate direction of car)
            arrowPoints = [self.realToPixelPos(self.mapToDraw.car.position), polygonPoints[1], polygonPoints[2]] #not as efficient as using the line below, but self.pos can vary
            oppositeColor = [255-self.carColor[0], 255-self.carColor[1], 255-self.carColor[1]]
            pygame.draw.polygon(self.window, oppositeColor, arrowPoints) #draw arrow
        else:
            print("car sprite TBD")
        
        if((time.time() - self.carHistTimer) > self.carHistTimeStep):
            self.carHistTimer = time.time()
            rearAxlePos = GF.distAnglePosToPos(self.mapToDraw.car.length/2, GF.radInv(self.mapToDraw.car.angle), self.mapToDraw.car.position)
            if((GF.distSqrdBetwPos(self.carHistPoints[-1][2], rearAxlePos) > self.carHistMinSquaredDistThresh) if (len(self.carHistPoints) > 1) else True):
                self.carHistPoints.append([[self.mapToDraw.car.position[0] + offsets[0][0], self.mapToDraw.car.position[1] + offsets[0][1]], [self.mapToDraw.car.position[0] - offsets[1][0], self.mapToDraw.car.position[1] - offsets[1][1]], rearAxlePos])
                if(len(self.carHistPoints) > self.carHistMaxLen):
                    self.carHistPoints.pop(0)
        
        if(len(self.carHistPoints) > 1):
            for i in range(1, len(self.carHistPoints)):
                for j in range(len(self.carHistPoints[i])):
                    pygame.draw.line(self.window, [200, 200, 200], self.realToPixelPos(self.carHistPoints[i-1][j]), self.realToPixelPos(self.carHistPoints[i][j]), 1)
    
    def drawMouseCone(self, drawPossibleConnections=True, drawConnectionThresholdCircle=False):
        if(self.mouseCone is not None): #if there is a floating cone to be drawn
            conePixelDiam = Map.Cone.coneDiam * self.sizeScale
            conePos = pygame.mouse.get_pos() #update position to match mouse position
            if(self.isInsideWindowPixels(conePos)): #should always be true, right?
                coneColor = self.rightConeColor if self.mouseCone else self.leftConeColor
                overlapsCone, overlappingCone = self.mapToDraw.overlapConeCheck(self.pixelsToRealPos(conePos))
                if(overlapsCone and drawPossibleConnections and self.coneConnecterPresent): #if mouse is hovering over existing cone
                    coneColor = self.rightConeColor if overlappingCone.LorR else self.leftConeColor #overlapping cone might have other cone color
                    nearbyConeList = self.mapToDraw.distanceToConeSquared(overlappingCone.position, self.mapToDraw.right_cone_list if overlappingCone.LorR else self.mapToDraw.left_cone_list, False, [overlappingCone.ID], self.mapToDraw.coneConnectionThresholdSquared, 'EXCL_DUBL_CONN', [overlappingCone.ID])
                    overlappingConePixelPos = self.realToPixelPos(overlappingCone.position)
                    for cone in nearbyConeList:
                        pygame.draw.line(self.window, coneColor, overlappingConePixelPos, self.realToPixelPos(cone[0].position), int(self.coneLineWidth/2))
                else:
                    if(drawPossibleConnections and self.coneConnecterPresent):
                        nearbyConeList = self.mapToDraw.distanceToConeSquared(self.pixelsToRealPos(conePos), self.mapToDraw.right_cone_list if self.mouseCone else self.mapToDraw.left_cone_list, False, [], self.mapToDraw.coneConnectionThresholdSquared, 'EXCL_DUBL_CONN', [])
                        for cone in nearbyConeList:
                            pygame.draw.line(self.window, coneColor, conePos, self.realToPixelPos(cone[0].position), int(self.coneLineWidth/2))
                    #pygame.draw.circle(self.window, coneColor, [int(conePos[0]), int(conePos[1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
                    conePos = GF.ASA(-(conePixelDiam/2), conePos) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
                    pygame.draw.ellipse(self.window, coneColor, [conePos, [conePixelDiam, conePixelDiam]]) #draw cone
                if(drawConnectionThresholdCircle):
                    pygame.draw.circle(self.window, coneColor, [int(conePos[0]), int(conePos[1])], self.mapToDraw.coneConnectionThreshold * self.sizeScale, self.coneLineWidth) #draw circle with coneConnectionThreshold radius 
    
    def drawDebugLines(self):
         #debugLines structure: [pos,pos, color_index (0-2)]
        for debugLine in self.debugLines:
            if(abs(debugLine[0]) == 2):
                if(debugLine[0] == 2):
                    pixelRactSize = debugLine[2][0] * self.sizeScale
                    debugLine[1] = GF.ASA(-(pixelRactSize), debugLine[1])
                    pixelRactSize *= 2
                    debugLine[2][0] = pixelRactSize
                    debugLine[0] = -2
                pygame.draw.arc(self.window, self.debugLineColors[(debugLine[3] if (len(debugLine)==4) else 0)], [debugLine[1], [debugLine[2][0], debugLine[2][0]]], debugLine[2][1], debugLine[2][2], self.debugLineWidth)
            else:
                if(debugLine[0] == 1):
                    secondPos = self.realToPixelPos(GF.distAnglePosToPos(debugLine[2][0], debugLine[2][1], self.pixelsToRealPos(debugLine[1]))) #convert
                    debugLine[2] = secondPos
                    debugLine[0] = -1
                pygame.draw.line(self.window, self.debugLineColors[(debugLine[3] if (len(debugLine)==4) else 0)], debugLine[1], debugLine[2], self.debugLineWidth)
    
    def updateViewOffset(self, mousePos=None): #screen dragging
        if(self.movingViewOffset):
            if(mousePos is None):
                mousePos = pygame.mouse.get_pos()
            mouseDelta = [] #init var
            if(self.invertYaxis):
                mouseDelta = [float(mousePos[0] - self.movingViewOffsetMouseStart[0]), float(self.movingViewOffsetMouseStart[1] - mousePos[1])]
            else:
                mouseDelta = [float(mousePos[0] - self.movingViewOffsetMouseStart[0]), float(mousePos[1] - self.movingViewOffsetMouseStart[1])]
            self.viewOffset[0] = self.prevViewOffset[0] + (mouseDelta[0]/self.sizeScale)
            self.viewOffset[1] = self.prevViewOffset[1] + (mouseDelta[1]/self.sizeScale)
    
    def redraw(self):
        self.updateViewOffset()
        self.background()
        self.drawCones(True) #boolean parameter is whether to draw lines between connected cones (track bounds) or not
        self.drawPathLines(self.drawTargetConeLines, True) #boolean parameters are whether to draw the lines between cones (not the line the car follows) and whether to draw circles (conesized ellipses) on the center points of path lines respectively
        self.drawFinishLine()
        self.drawCar()
        #debug and UI
        self.drawMouseCone(True, False)
        self.drawDebugLines()
    
    def updateWindowSize(self, drawSize=[1200, 600], drawOffset=[0,0], sizeScale=-1, autoMatchSizeScale=True):
        if(sizeScale > 0):
            self.sizeScale = sizeScale
        elif(autoMatchSizeScale):
            self.sizeScale = min(drawSize[0]/self.drawSize[0], drawSize[1]/self.drawSize[1]) * self.sizeScale #auto update sizeScale to match previous size
        self.drawSize = (int(drawSize[0]), int(drawSize[1]))
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1]))





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


global windowKeepRunning, windowStarted
windowStarted = False
windowKeepRunning = False

global pygamesimInputLast, oldWindowSize
pygamesimInputLast = None #to be filled
oldWindowSize = []


def pygameInit():
    pygame.init()
    pygame.font.init()
    global window, oldWindowSize
    window = pygame.display.set_mode([1200, 600], pygame.RESIZABLE)
    oldWindowSize = window.get_size()
    pygame.display.set_caption("(pygame) selfdriving sim")
    global windowKeepRunning, windowStarted
    windowStarted = True
    windowKeepRunning = True

def pygameEnd():
    global windowStarted
    if(windowStarted): #if the window never started, quit might error out or something stupid
        print("quitting pygame window...")
        pygame.quit()

def frameRefresh():
    pygame.display.flip() #send (finished) frame to display

def handleMousePress(pygamesimInput, buttonDown, button, pos, eventToHandle):
    if(buttonDown and ((button == 1) or (button == 3))): #left/rigth mouse button pressed (down)
        pygame.event.set_grab(1)
        leftOrRight = (True if (button == 3) else False)
        if(not pygame.key.get_pressed()[pygame.K_r]):
            pygamesimInput.mouseCone = leftOrRight
        if(pygame.key.get_pressed()[pygame.K_f]):
            pygame.mouse.set_cursor(flagCurs16Data[0], flagCurs16Data[1], flagCurs16Data[2], flagCurs16Data[3]) #smaller flag cursor
    elif((button == 1) or (button == 3)): #if left/right mouse button released
        leftOrRight = (True if (button == 3) else False)
        pygame.event.set_grab(0)
        pygamesimInput.mouseCone = None
        if(pygame.key.get_pressed()[pygame.K_r]):
            overlaps, overlappingCone = pygamesimInput.mapToDraw.overlapConeCheck(pygamesimInput.pixelsToRealPos(pos))
            if(overlaps):
                deleting = True
                for target in pygamesimInput.mapToDraw.target_list: #look through the entire list of targets (you could also just look at self.mapToDraw.target_list[-2])
                    if(overlappingCone.ID == target.coneConData.cones[overlappingCone.LorR].ID): #if the only connected cone is ALSO (already) in the target_list, then you cant make any more path
                        print("can't delete cone, it's in pathList") #just a lot easier, not impossible
                        deleting = False
                if(deleting):
                    print("deleting cone:", overlappingCone.ID)
                    for connectedCone in overlappingCone.connections:
                        if(len(connectedCone.coneConData) > 0): #it's always a list, but an empty one if coneConnecter is not used
                            connectedCone.coneConData.pop((0 if (connectedCone.connections[0].ID == overlappingCone.ID) else 1))
                        connectedCone.connections.pop((0 if (connectedCone.connections[0].ID == overlappingCone.ID) else 1))
                    listToRemoveFrom = (pygamesimInput.mapToDraw.right_cone_list if overlappingCone.LorR else pygamesimInput.mapToDraw.left_cone_list)
                    listToRemoveFrom.pop(GF.findIndexByClassAttr(listToRemoveFrom, 'ID', overlappingCone.ID))
        else:
            if((len(pygamesimInput.mapToDraw.finish_line_cones) < 2) if pygame.key.get_pressed()[pygame.K_f] else True):
                posToPlace = pygamesimInput.pixelsToRealPos(pos)
                overlaps, overlappingCone = pygamesimInput.mapToDraw.overlapConeCheck(posToPlace)
                if(overlaps):
                    if(pygame.key.get_pressed()[pygame.K_f]):
                        if((pygamesimInput.mapToDraw.finish_line_cones[0].LorR != overlappingCone.LorR) if (len(pygamesimInput.mapToDraw.finish_line_cones) > 0) else True):
                            overlappingCone.isFinish = True
                            pygamesimInput.mapToDraw.finish_line_cones.append(overlappingCone)
                        else:
                            print("can't set (existing) cone as finish, there's aready a "+("right" if leftOrRight else "left")+"-sided finish cone")
                    elif(pygamesimInput.coneConnecterPresent):
                        pygamesimInput.mapToDraw.connectCone(overlappingCone)
                else:
                    newConeID = GF.findMaxAttrIndex((pygamesimInput.mapToDraw.right_cone_list + pygamesimInput.mapToDraw.left_cone_list), 'ID')[1]
                    aNewCone = Map.Cone(newConeID+1, posToPlace, leftOrRight, pygame.key.get_pressed()[pygame.K_f])
                    if(((pygamesimInput.mapToDraw.finish_line_cones[0].LorR != leftOrRight) if (len(pygamesimInput.mapToDraw.finish_line_cones) > 0) else True) if pygame.key.get_pressed()[pygame.K_f] else True):
                        coneListToAppend = (pygamesimInput.mapToDraw.right_cone_list if leftOrRight else pygamesimInput.mapToDraw.left_cone_list)
                        coneListToAppend.append(aNewCone)
                        if(pygame.key.get_pressed()[pygame.K_f]):
                            pygamesimInput.mapToDraw.finish_line_cones.append(aNewCone)
                        if(pygame.key.get_pressed()[pygame.K_LSHIFT] and pygamesimInput.coneConnecterPresent):
                            pygamesimInput.mapToDraw.connectCone(aNewCone)
    elif(button==2): #middle mouse button
        if(buttonDown): #mouse pressed down
            if(not pygamesimInput.carCam):
                pygame.event.set_grab(1)
                pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
                pygamesimInput.movingViewOffset = True
                pygamesimInput.movingViewOffsetMouseStart = pygame.mouse.get_pos()
                pygamesimInput.prevViewOffset = (pygamesimInput.viewOffset[0], pygamesimInput.viewOffset[1])
        else:           #mouse released
            pygame.event.set_grab(0)
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
            pygamesimInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
            pygamesimInput.movingViewOffset = False

def handleKeyPress(pygamesimInput, keyDown, key, eventToHandle):
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
    if(key==pygame.K_r): # r
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
    elif(key==pygame.K_p): # p
        if(keyDown and pygamesimInput.pathFinderPresent):
            pygamesimInput.mapToDraw.makePath()
            # doesNothing = 0
            # while(pygamesimInput.mapToDraw.makePath()): #stops when path can no longer be advanced
            #     doesNothing += 1  # "python is so versitile, you can do anything" :) haha good joke
    elif(key==pygame.K_t): # t
        if(keyDown):
            pygamesimInput.drawTargetConeLines = not pygamesimInput.drawTargetConeLines #only has an effect if pyagmesimInput.pathFinderPresent == True
    elif(key==pygame.K_c): # c
        if(keyDown):
            pygamesimInput.carHistPoints = []
    elif(key==pygame.K_v): # v
        if(keyDown):
            pygamesimInput.carCam = not pygamesimInput.carCam
            if(pygamesimInput.carCam and pygamesimInput.movingViewOffset): #if you switched to carCam while you were moving viewOffset, just stop moving viewOffset (same as letting go of MMB)
                pygame.event.set_grab(0)
                pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
                pygamesimInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
                pygamesimInput.movingViewOffset = False

def currentPygamesimInput(pygamesimInputList, mousePos=None, demandMouseFocus=True): #if no pos is specified, retrieve it using get_pos()
    if(len(pygamesimInputList) > 1):
        if(mousePos is None):
            mousePos = pygame.mouse.get_pos()
        global pygamesimInputLast
        if(pygame.mouse.get_focused() or (not demandMouseFocus)):
            for pygamesimInput in pygamesimInputList:
                # localBoundries = [[pygamesimInput.drawOffset[0], pygamesimInput.drawOffset[1]], [pygamesimInput.drawSize[0], pygamesimInput.drawSize[1]]]
                # if(((mousePos[0]>=localBoundries[0][0]) and (mousePos[0]<(localBoundries[0][0]+localBoundries[1][0]))) and ((mousePos[1]>=localBoundries[0][1]) and (mousePos[0]<(localBoundries[0][1]+localBoundries[1][1])))):
                if(pygamesimInput.isInsideWindowPixels(mousePos)):
                    pygamesimInputLast = pygamesimInput
                    return(pygamesimInput)
        if(pygamesimInputLast is None): #if this is the first interaction
            pygamesimInputLast = pygamesimInputList[0]
        return(pygamesimInputLast)
    else:
        return(pygamesimInputList[0])

def handleWindowEvent(pygamesimInputList, eventToHandle):
    global window, oldWindowSize
    if(eventToHandle.type == pygame.QUIT):
        global windowKeepRunning
        windowKeepRunning = False #stop program (soon)
    
    elif(eventToHandle.type == pygame.VIDEORESIZE):
        newSize = eventToHandle.size
        if((oldWindowSize[0] != newSize[0]) or (oldWindowSize[1] != newSize[1])): #if new size is actually different
            print("video resize from", oldWindowSize, "to", newSize)
            correctedSize = [newSize[0], newSize[1]]
            window = pygame.display.set_mode(correctedSize, pygame.RESIZABLE)
            for pygamesimInput in pygamesimInputList:
                localNewSize = [int((pygamesimInput.drawSize[0]*correctedSize[0])/oldWindowSize[0]), int((pygamesimInput.drawSize[1]*correctedSize[1])/oldWindowSize[1])]
                localNewDrawPos = [int((pygamesimInput.drawOffset[0]*correctedSize[0])/oldWindowSize[0]), int((pygamesimInput.drawOffset[1]*correctedSize[1])/oldWindowSize[1])]
                pygamesimInput.updateWindowSize(localNewSize, localNewDrawPos, autoMatchSizeScale=False)
        oldWindowSize = window.get_size() #update size (get_size() returns tuple of (width, height))
    
    elif(eventToHandle.type == pygame.WINDOWSIZECHANGED): # pygame 2.0.1 compatible
        newSize = window.get_size()
        if((oldWindowSize[0] != newSize[0]) or (oldWindowSize[1] != newSize[1])): #if new size is actually different
            print("video resize from", oldWindowSize, "to", newSize)
            correctedSize = [newSize[0], newSize[1]]
            for pygamesimInput in pygamesimInputList:
                localNewSize = [int((pygamesimInput.drawSize[0]*correctedSize[0])/oldWindowSize[0]), int((pygamesimInput.drawSize[1]*correctedSize[1])/oldWindowSize[1])]
                localNewDrawPos = [int((pygamesimInput.drawOffset[0]*correctedSize[0])/oldWindowSize[0]), int((pygamesimInput.drawOffset[1]*correctedSize[1])/oldWindowSize[1])]
                pygamesimInput.updateWindowSize(localNewSize, localNewDrawPos, autoMatchSizeScale=False)
        oldWindowSize = window.get_size() #update size (get_size() returns tuple of (width, height))
    
    # elif(eventToHandle.type == pygame.DROPFILE): #drag and drop files to import them
    #     if((pygame.mouse.get_pos()[0] == 0) and (pygame.mouse.get_pos()[1] == 0) and (len(pygamesimInputList) > 1)):
    #         print("skipping file import, please make sure to select the pygame window beforehand or something")
    #     else:
    #         currentPygamesimInput(pygamesimInputList, None, False).importConeLog(eventToHandle.file, True) #note: drag and drop functionality is a little iffy for multisim applications
    
    elif((eventToHandle.type == pygame.MOUSEBUTTONDOWN) or (eventToHandle.type == pygame.MOUSEBUTTONUP)):
        #print("mouse press", eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos)
        handleMousePress(currentPygamesimInput(pygamesimInputList, eventToHandle.pos, True), eventToHandle.type == pygame.MOUSEBUTTONDOWN, eventToHandle.button, eventToHandle.pos, eventToHandle)
        
    elif((eventToHandle.type == pygame.KEYDOWN) or (eventToHandle.type == pygame.KEYUP)):
        #print("keypress:", eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, pygame.key.name(eventToHandle.key))
        handleKeyPress(currentPygamesimInput(pygamesimInputList, None, True), eventToHandle.type == pygame.KEYDOWN, eventToHandle.key, eventToHandle)
    
    elif(eventToHandle.type == pygame.MOUSEWHEEL): #scroll wheel (zooming / rotating)
        simToScale = currentPygamesimInput(pygamesimInputList, None, True)
        if(pygame.key.get_pressed()[pygame.K_LCTRL] and simToScale.carCam): #if holding (left) CTRL while in carCam mode, rotate the view
            simToScale.carCamOrient += (eventToHandle.y * np.pi/16)
        else:
            dif = [simToScale.drawSize[0]/simToScale.sizeScale, simToScale.drawSize[1]/simToScale.sizeScale]
            simToScale.sizeScale += eventToHandle.y #zooming
            #if(not simToScale.carCam): #viewOffset is not used in carCam mode, but it won't hurt to change it anyway
            dif[0] -= (simToScale.drawSize[0]/simToScale.sizeScale)
            dif[1] -= (simToScale.drawSize[1]/simToScale.sizeScale)
            simToScale.viewOffset[0] -= dif[0]/2 #equalizes from the zoom to 'happen' from the middle of the screen
            simToScale.viewOffset[1] -= dif[1]/2

def handleAllWindowEvents(pygamesimInput): #input can be pygamesim object, 1D list of pygamesim objects or 2D list of pygamesim objects
    pygamesimInputList = []
    if(type(pygamesimInput) is list):
        if(len(pygamesimInput) > 0):
            for entry in pygamesimInput:
                if(type(entry) is list):
                    for subEntry in entry:
                        pygamesimInputList.append(subEntry) #2D lists
                else:
                    pygamesimInputList.append(entry) #1D lists
    else:
        pygamesimInputList = [pygamesimInput] #convert to 1-sizes array
    #pygamesimInputList = pygamesimInput #assume input is list of pygamesims
    if(len(pygamesimInputList) < 1):
        print("len(pygamesimInputList) < 1")
        global windowKeepRunning
        windowKeepRunning = False
        pygame.event.pump()
        return()
    for eventToHandle in pygame.event.get(): #handle all events
        handleWindowEvent(pygamesimInputList, eventToHandle)
    
    #the manual keyboard driving (tacked on here, because doing it with the event system would require more variables, and this is temporary anyway)
    simToDrive = currentPygamesimInput(pygamesimInputList, demandMouseFocus=False) #get the active sim within the window
    carToDrive = simToDrive.mapToDraw.car
    pressedKeyList = pygame.key.get_pressed()
    deltaTime = time.time() - simToDrive.carKeyboardControlTimer
    simToDrive.carKeyboardControlTimer = time.time()
    speedAccelVal = 10.0 * deltaTime
    steerAccelVal = 1.5 * deltaTime
    #first for speed
    if(pressedKeyList[pygame.K_UP]): #accelerate button
        carToDrive.velocity += speedAccelVal #accelerate
    elif(pressedKeyList[pygame.K_DOWN]): #brake/reverse button
        if(carToDrive.velocity > (speedAccelVal*3)): #positive speed
            carToDrive.velocity -= speedAccelVal * 2 #fast brake
        else:                               #near-zero or negative speed
            carToDrive.velocity -= speedAccelVal * 0.5 #reverse accelerate
    else:                           #neither buttons
        if(carToDrive.velocity > speedAccelVal): #positive speed
            carToDrive.velocity -= speedAccelVal/2 #slow brake
        elif(carToDrive.velocity < -speedAccelVal): #negative speed
            carToDrive.velocity += speedAccelVal #brake
        else:                           #near-zero speed
            carToDrive.velocity = 0
    carToDrive.velocity = max(-4, min(8, carToDrive.velocity)) #limit speed
    #now for steering
    if(pressedKeyList[pygame.K_LEFT] and (not pressedKeyList[pygame.K_RIGHT])):
        carToDrive.steering += steerAccelVal
    elif(pressedKeyList[pygame.K_RIGHT] and (not pressedKeyList[pygame.K_LEFT])):
        carToDrive.steering -= steerAccelVal
    else:
        if(carToDrive.steering > steerAccelVal):
            carToDrive.steering -= steerAccelVal*2.0
        elif(carToDrive.steering < -steerAccelVal):
            carToDrive.steering += steerAccelVal*2.0
        else:
            carToDrive.steering = 0
    carToDrive.steering = max(np.deg2rad(-25), min(np.deg2rad(25), carToDrive.steering)) #limit speed







