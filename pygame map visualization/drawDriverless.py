import pygame       #python game library, used for the visualization
import numpy as np  #general math library
import time         #used for (temporary) driving math in raceCar() class

import os #only used for loading the car sprite (os is used to get the filepath)
from PIL import Image, ImageDraw #python image library, only used for drawing car headlights

from Map import Map
import generalFunctions as GF #(homemade) some useful functions for everyday ease of use




class pygameDrawer():
    """a class to draw Map objects (also includes drawing of UI some elements for manually making tracks)"""
    def __init__(self, mapToDraw, window, drawSize=(1200,600), drawOffset=(0,0), carCamOrient=0, sizeScale=120, startWithCarCam=False, invertYaxis=True):
        self.window = window #pass on the window object (pygame)
        self.drawSize = (int(drawSize[0]),int(drawSize[1])) #width and height of the display area (does not have to be 100% of the window)
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1])) #draw position offset, (0,0) is topleft
        self.viewOffset = [0.0, 0.0] #'camera' view offsets, changing this affects the real part of realToPixelPos()
        self.carCamOrient = carCamOrient #orientation of the car (and therefore everything) on the screen. 0 is towards the right
        self.sizeScale = sizeScale #pixels per meter
        self.carCam = startWithCarCam #it's either carCam (car-centered cam, with rotating but no viewOffset), or regular cam (with viewOffset, but no rotating)
        self.invertYaxis = invertYaxis #pygame has pixel(0,0) in the topleft, so this just flips the y-axis when drawing things
        
        self.mapToDraw = mapToDraw
        
        self.bgColor = [50,50,50] #grey
        self.fontSize = 30
        self.pygameFont = pygame.font.Font(None, self.fontSize)
        
        self.finishLineColor = [255,40,0]
        self.finishLineWidth = 2 #pixels wide
        
        self.leftConeColor = [255,255,0] #yellow
        self.rightConeColor = [0,50,255] #dark blue
        self.coneLineWidth = 2 #pixels wide
        
        self.pathColor = [0,220,255] #light blue
        self.pathLineWidth = 2 #pixels wide
        #self.pathCenterPixelDiam = 
        self.pathPointDiam = Map.Cone.coneDiam / 2
        
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
        self.carPolygonMode = False #if no sprite is present, this can be used to draw a simple car
        try: #if the sprite doesnt load (usually because it's missing), thats fine
            current_dir = os.path.dirname(os.path.abspath(__file__)) #get directory name of current sketch
            self.car_image = pygame.image.load(os.path.join(current_dir, "carSprite.png"))
        except Exception as excep:
            print("couldn't load car sprite ", excep)
            print("using simple polygon car instead")
            self.carPolygonMode = True
        self.headlights = False #only has an effect if carPolygonMode=False
        
        self.drawQubicSplines = True #only has an effect if pathPlanningPresent=True
        
        #these should really be initialized in a class that makes use of pygameDrawer, but this avoids errors and is harmless (as long as pygameDrawer.__init__() is run BEFORE initializing thsese in the child class)
        self.coneConnecterPresent = False
        self.pathFinderPresent = False
        self.pathPlanningPresent = False
        self.SLAMPresent = False
        
        self.isRemote = False  #maybe these shouldn't be initialised here, but this does avoid errors
        self.remoteUIsender = None
        self.remoteFPS = 5 #this should be initialised here though
        
        self.drawTargetConeLines = False #just for UI purposes, to toggle between showing and not showing how the targets are made
        
        self.carHistTimer = self.mapToDraw.clock()
        self.carHistPoints = []
        self.carHistTimeStep = 0.15
        self.carHistMinSquaredDistThresh = 0.1**2 #only save new positions if the car is moving
        self.carHistMaxLen = 200
        
        self.FPStimer = time.time()
        self.FPSdata = []
        self.FPSdisplayInterval = 0.25
        self.FPSdisplayTimer = time.time()
        self.FPSrenderedFonts = []
        
        self.carKeyboardControlTimer = time.time() #ONLY USED FOR KEYBOARD DRIVING (commented out)
        
        try:
            self.viewOffset = [(-self.car.position[0]) + ((self.drawSize[0]/self.sizeScale)/2), (-self.car.position[1]) + ((self.drawSize[1]/self.sizeScale)/2)]
        except Exception as theExcept:
            print("couldn't set viewOffset to car pos:", theExcept)
    
    #pixel conversion functions (the most important functions in here)
    def pixelsToRealPos(self, pixelPos):
        """return a (real) position for a given pixel position (usually mouse position)
            (mostly used for UI)"""
        if(self.carCam):
            dist = 0; angle = 0; #init var
            if(self.invertYaxis):
                dist, angle = GF.distAngleBetwPos(np.array([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2]), np.array([pixelPos[0], self.drawOffset[1]+(self.drawOffset[1]+self.drawSize[1])-pixelPos[1]])) #get distance to, and angle with respect to, center of the screen (car)
            else:
                dist, angle = GF.distAngleBetwPos(np.array([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2]), np.array(pixelPos)) #get distance to, and angle with respect to, center of the screen (car)
            return(GF.distAnglePosToPos(dist/self.sizeScale, angle+self.mapToDraw.car.angle-self.carCamOrient, self.mapToDraw.car.position)) #use converted dist, correctly offset angle & the real car pos to get a new real point
        else:
            if(self.invertYaxis):
                return (np.array([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((self.drawSize[1]-pixelPos[1]+self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]]))
            else:
                return (np.array([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((pixelPos[1]-self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]]))
    
    def realToPixelPos(self, realPos: np.ndarray):
        """return the pixel-position (for pygame) for a given (real) position"""
        if(self.carCam):
            dist, angle = GF.distAngleBetwPos(self.mapToDraw.car.position, np.array(realPos)) #get distance to, and angle with respect to, car
            shiftedPixelPos = GF.distAnglePosToPos(dist*self.sizeScale, angle-self.mapToDraw.car.angle+self.carCamOrient, np.array([self.drawOffset[0]+self.drawSize[0]/2, self.drawOffset[1]+self.drawSize[1]/2])) #calculate new (pixel) pos from the car pos, at the same distance, and the angle, plus the angle that the entire scene is shifted
            if(self.invertYaxis):
                return(np.array([shiftedPixelPos[0], self.drawOffset[1]+((self.drawOffset[1]+self.drawSize[1])-shiftedPixelPos[1])])) #invert Y-axis for normal (0,0) at bottomleft display
            else:
                return(shiftedPixelPos)
        else:
            if(self.invertYaxis):
                return(np.array([((realPos[0]+self.viewOffset[0])*self.sizeScale)+self.drawOffset[0], self.drawSize[1]-((realPos[1]+self.viewOffset[1])*self.sizeScale)+self.drawOffset[1]])) #invert Y-axis for normal (0,0) at bottomleft display
            else:
                return(np.array([((realPos[0]+self.viewOffset[0])*self.sizeScale)+self.drawOffset[0], ((realPos[1]+self.viewOffset[1])*self.sizeScale)+self.drawOffset[1]]))
    
    #check if things need to be drawn at all
    def isInsideWindowPixels(self, pixelPos):
        """whether or not a pixel-position is inside the window"""
        return((pixelPos[0] < (self.drawSize[0] + self.drawOffset[0])) and (pixelPos[0] > self.drawOffset[0]) and (pixelPos[1] < (self.drawSize[1] + self.drawOffset[1])) and (pixelPos[1] > self.drawOffset[1]))
    
    def isInsideWindowReal(self, realPos: np.ndarray):
        """whether or not a (real) position is inside the window (note: not computationally efficient)"""
        return(self.isInsideWindowPixels(self.realToPixelPos(realPos))) #not very efficient, but simple
    
    #drawing functions
    def background(self):
        """draw the background"""
        self.window.fill(self.bgColor, (self.drawOffset[0], self.drawOffset[1], self.drawSize[0], self.drawSize[1])) #dont fill entire screen, just this pygamesim's area (allowing for multiple sims in one window)
    
    def drawFPScounter(self):
        """draw a little Frames Per Second counter in the corner to show program performance"""
        newTime = time.time()
        if((newTime - self.FPStimer)>0): #avoid divide by 0
            self.FPSdata.append(round(1/(newTime-self.FPStimer), 1))
        self.FPStimer = newTime #save for next time
        if((newTime - self.FPSdisplayTimer)>self.FPSdisplayInterval):
            self.FPSdisplayTimer = newTime
            FPSstrings = []
            if(len(self.FPSdata)>0):
                FPSstrings.append(str(round(GF.average(np.array(self.FPSdata)), 1))) #average FPS
                FPSstrings.append(str(min(self.FPSdata)))                  #minimum FPS
                FPSstrings.append(str(max(self.FPSdata)))                  #maximum FPS
                self.FPSdata.sort()
                FPSstrings.append(str(self.FPSdata[int((len(self.FPSdata)-1)/2)])) #median FPS
                #print("FPS:", round(GF.average(np.array(self.FPSdata)), 1), min(self.FPSdata), max(self.FPSdata), self.FPSdata[int((len(self.FPSdata)-1)/2)])
            else:
                FPSstrings = ["inf"]
                #print("FPS: inf")
            self.FPSdata = []
            self.FPSrenderedFonts = []
            for FPSstr in FPSstrings:
                self.FPSrenderedFonts.append(self.pygameFont.render(FPSstr, False, [255-self.bgColor[0], 255-self.bgColor[1], 255-self.bgColor[2]], self.bgColor)) #render string (only 1 line per render allowed), no antialiasing, text color opposite of bgColor, background = bgColor
        for i in range(len(self.FPSrenderedFonts)):
            self.window.blit(self.FPSrenderedFonts[i], [self.drawOffset[0]+ self.drawSize[0]-5-self.FPSrenderedFonts[i].get_width(),self.drawOffset[1]+ 5+(i*self.fontSize)])
    
    def drawCones(self, drawLines=True):
        """draw the cones and their connections
            (if enabled, draw qubic splines instead of direct connections)"""
        conePixelDiam = Map.Cone.coneDiam * self.sizeScale
        drawnLineList = [] #[ [ID, ID], ] just a list of drawn lines by ID
        for LorR in range(2):
            coneColor = self.rightConeColor if LorR else self.leftConeColor
            for cone in (self.mapToDraw.right_cone_list if LorR else self.mapToDraw.left_cone_list):
                conePos = self.realToPixelPos(cone.position) #convert to pixel position
                ## draw the lines between cones
                if(drawLines and (not self.drawQubicSplines)):
                    alreadyDrawn = [False, False]
                    for drawnLine in drawnLineList:
                        for i in range(len(cone.connections)):
                            if((cone.ID in drawnLine) and (cone.connections[i].ID in drawnLine)):
                                alreadyDrawn[i] = True
                    for i in range(len(cone.connections)):
                        if(not alreadyDrawn[i]):
                            pygame.draw.line(self.window, coneColor, conePos, self.realToPixelPos(cone.connections[i].position), self.coneLineWidth)
                            drawnLineList.append([cone.ID, cone.connections[i].ID]) #put established 'back' connection in list of drawn 
                ## now draw the cone itself
                #pygame.draw.circle(self.window, coneColor, [int(conePos[0]), int(conePos[1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
                conePos = GF.ASA(-(conePixelDiam/2), conePos) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
                pygame.draw.ellipse(self.window, coneColor, [conePos, [conePixelDiam, conePixelDiam]]) #draw cone
            if(self.pathPlanningPresent and self.drawQubicSplines):
                splinePointPixelDiam = self.splinePointDiam * self.sizeScale
                splineList = (self.right_spline if LorR else self.left_spline)
                for i in range(len(splineList[0])):
                    splinePointPos = self.realToPixelPos([splineList[0][i], splineList[1][i]])
                    pygame.draw.ellipse(self.window, coneColor, [GF.ASA(-(splinePointPixelDiam/2), splinePointPos), [splinePointPixelDiam, splinePointPixelDiam]]) #draw cone
                    if(i > 0):#if more than one spline point exists (and the forloop is past the first one)
                        lastSplinePointPos = self.realToPixelPos([splineList[0][i-1], splineList[1][i-1]])
                        pygame.draw.line(self.window, coneColor, lastSplinePointPos, splinePointPos, self.coneLineWidth) #line from center pos to center pos
                ## drawing the last little line to complete the full-circle is rather difficult in the current system, so i won't bother
    
    def drawPathLines(self, drawPoints=False, drawConeLines=True):
        """draw the path (target_list)
            (if enabled, draw qubic splines instead of direct connections)"""
        # target_list content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        pathCenterPixelDiam = self.pathPointDiam * self.sizeScale
        if(self.pathPlanningPresent and self.drawQubicSplines):
            splinePointPixelDiam = self.splinePointDiam * self.sizeScale
            for i in range(len(self.path_midpoints_spline[0])):
                targetPos = self.realToPixelPos([self.path_midpoints_spline[0][i], self.path_midpoints_spline[1][i]])
                if(drawPoints):
                    pygame.draw.ellipse(self.window, self.pathColor, [GF.ASA(-(splinePointPixelDiam/2), targetPos), [splinePointPixelDiam, splinePointPixelDiam]]) #draw spline point
                if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                    lastTargetPos = self.realToPixelPos([self.path_midpoints_spline[0][i-1], self.path_midpoints_spline[1][i-1]])
                    pygame.draw.line(self.window, self.pathColor, lastTargetPos, targetPos, self.pathLineWidth) #line from center pos to center pos
            # if(self.mapToDraw.targets_full_circle and (len(self.path_midpoints_spline[0]) > 1)): #note: this depends on mapToDraw, which path_midpoints_spline might not, be careful
            #     startingPos = self.realToPixelPos([self.path_midpoints_spline[0][0], self.path_midpoints_spline[1][0]])
            #     endingPos = self.realToPixelPos([self.path_midpoints_spline[0][-1], self.path_midpoints_spline[1][-1]])
            #     pygame.draw.line(self.window, self.pathColor, endingPos, startingPos, self.pathLineWidth) #line that loops around to start
        else:
            for i in range(len(self.mapToDraw.target_list)):
                #if(self.isInsideWindowReal(self.mapToDraw.target_list[i].position)):
                if(drawPoints):
                    #centerPixelPos = self.realToPixelPos(self.mapToDraw.target_list[i][0])
                    #pygame.draw.circle(self.window, self.pathColor, [int(centerPixelPos[0]), int(centerPixelPos[1])], int(pathCenterPixelDiam/2)) #draw center point (as filled circle, not ellipse)
                    pygame.draw.ellipse(self.window, self.pathColor, [GF.ASA(-(pathCenterPixelDiam/2), self.realToPixelPos(self.mapToDraw.target_list[i].position)), [pathCenterPixelDiam, pathCenterPixelDiam]]) #draw center point
                if(drawConeLines and (self.mapToDraw.target_list[i].coneConData is not None)): #instead of checking if pathFinderPresent, we can just check if the data is there (also more isRemote friendly)
                    pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.mapToDraw.target_list[i].coneConData.cones[0].position), self.realToPixelPos(self.mapToDraw.target_list[i].coneConData.cones[1].position), self.pathLineWidth) #line from left cone to right cone
                if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                    #draw line between center points of current pathline and previous pathline (to make a line that the car should (sort of) follow)
                    pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.mapToDraw.target_list[i-1].position), self.realToPixelPos(self.mapToDraw.target_list[i].position), self.pathLineWidth) #line from center pos to center pos
            if(self.mapToDraw.targets_full_circle):
                pygame.draw.line(self.window, self.pathColor, self.realToPixelPos(self.mapToDraw.target_list[-1].position), self.realToPixelPos(self.mapToDraw.target_list[0].position), self.pathLineWidth) #line that loops around to start
    
    def drawFinishLine(self):
        """draw finish line (between finish_line_cones)"""
        if(len(self.mapToDraw.finish_line_cones) >= 2):
            pygame.draw.line(self.window, self.finishLineColor, self.realToPixelPos(self.mapToDraw.finish_line_cones[0].position), self.realToPixelPos(self.mapToDraw.finish_line_cones[1].position), self.finishLineWidth)
    
    def drawCar(self):
        """draw car sprite
            (or a simple polygon instead, if sprite failed/disabled)"""
        ## drawing is currently done by calculating the position of the corners and drawing a polygon with those points. Not efficient, not pretty, but fun
        carToDraw = self.mapToDraw.car
        chassisCenter = GF.distAnglePosToPos(carToDraw.chassis_length_offset, carToDraw.angle, carToDraw.position)
        if(self.carPolygonMode):
            if(self.carPointRadius is None):
                self.carPointRadius = (((carToDraw.chassis_width**2)+(carToDraw.chassis_length**2))**0.5)/2 #Pythagoras
                self.carPointAngle = np.arctan2(carToDraw.chassis_width, carToDraw.chassis_length) #this is used to make corner point for polygon
            polygonPoints = []
            offsets = [[np.cos(self.carPointAngle+carToDraw.angle) * self.carPointRadius, np.sin(self.carPointAngle+carToDraw.angle) * self.carPointRadius],
                        [np.cos(np.pi-self.carPointAngle+carToDraw.angle) * self.carPointRadius, np.sin(np.pi-self.carPointAngle+carToDraw.angle) * self.carPointRadius]]
            polygonPoints.append(self.realToPixelPos([chassisCenter[0] + offsets[0][0], chassisCenter[1] + offsets[0][1]])) #front left
            polygonPoints.append(self.realToPixelPos([chassisCenter[0] + offsets[1][0], chassisCenter[1] + offsets[1][1]])) #back left
            polygonPoints.append(self.realToPixelPos([chassisCenter[0] - offsets[0][0], chassisCenter[1] - offsets[0][1]])) #back right
            polygonPoints.append(self.realToPixelPos([chassisCenter[0] - offsets[1][0], chassisCenter[1] - offsets[1][1]])) #front right
            pygame.draw.polygon(self.window, self.carColor, polygonPoints) #draw car
            #arrow drawing (not needed, just handy to indicate direction of car)
            arrowPoints = [self.realToPixelPos(chassisCenter), polygonPoints[1], polygonPoints[2]] #not as efficient as using the line below, but self.pos can vary
            oppositeColor = [255-self.carColor[0], 255-self.carColor[1], 255-self.carColor[1]]
            pygame.draw.polygon(self.window, oppositeColor, arrowPoints) #draw arrow
        else:
            if(self.headlights):# draw headlights (first, to not overlap car sprite)
                headlightsImageSize = int(2.0*2*self.sizeScale) #2.0 is arbitrary for now, to be replaced with apprixate camera/sensor range
                headlightsImage = Image.new("RGBA", (headlightsImageSize, headlightsImageSize))
                headlightsImageDrawObj = ImageDraw.Draw(headlightsImage)
                #pil_draw.arc((0, 0, pil_size-1, pil_size-1), 0, 270, fill=RED)
                headlightsCenterAngle = -1 * np.rad2deg((self.carCamOrient) if self.carCam else (carToDraw.angle))
                headlightsImageDrawObj.pieslice((0, 0, headlightsImageSize-1, headlightsImageSize-1), headlightsCenterAngle-60, headlightsCenterAngle+60, fill= (55, 55, 35))
                headlightsImage = pygame.image.fromstring(headlightsImage.tobytes(), headlightsImage.size, headlightsImage.mode)
                headlightsImage_rect = headlightsImage.get_rect(center=self.realToPixelPos(chassisCenter))
                self.window.blit(headlightsImage, headlightsImage_rect)
            scaledCarSprite = pygame.transform.scale(self.car_image, (int(carToDraw.chassis_length*self.sizeScale), int(carToDraw.chassis_width*self.sizeScale))) #note: height (length) and width are switched because an angle of 0 is at 3 o'clock, (and the car sprite is loaded like that)
            rotatedCarSprite = pygame.transform.rotate(scaledCarSprite, np.rad2deg((self.carCamOrient) if self.carCam else (carToDraw.angle)))
            rotatedCarRect = rotatedCarSprite.get_rect()
            carPos = self.realToPixelPos(chassisCenter)
            carPos = (carPos[0] - (rotatedCarRect.width / 2), carPos[1] - (rotatedCarRect.height / 2))
            #pygame.draw.rect(self.window, (200,200,200), (carPos, (rotatedCarRect.chassis_width, rotatedCarRect.height))) #draws a little box around the car sprite (just for debug)
            self.window.blit(rotatedCarSprite, carPos) #draw car
        
        if((carToDraw.pathFolData is not None) and self.pathPlanningPresent):
            if(carToDraw.pathFolData.nextTarget is not None):
                targetPos = self.realToPixelPos(carToDraw.pathFolData.nextTarget.position)
                carPos = self.realToPixelPos(carToDraw.position)
                pygame.draw.line(self.window, [255, 255, 255], carPos, targetPos, 1)
        
        if((self.mapToDraw.clock() - self.carHistTimer) > self.carHistTimeStep):
            self.carHistTimer = self.mapToDraw.clock()
            rearAxlePos = carToDraw.getRearAxlePos()
            if((GF.distSqrdBetwPos(np.array(self.carHistPoints[-1][0]), rearAxlePos) > self.carHistMinSquaredDistThresh) if (len(self.carHistPoints) > 1) else True):
                self.carHistPoints.append([rearAxlePos]) #you can add any number of points to store, as long as the first one is rearAxlePos
                if(len(self.carHistPoints) > self.carHistMaxLen):
                    self.carHistPoints.pop(0)
        
        if(len(self.carHistPoints) > 1):
            for i in range(1, len(self.carHistPoints)):
                for j in range(len(self.carHistPoints[i])):
                    pygame.draw.line(self.window, [200, 200, 200], self.realToPixelPos(self.carHistPoints[i-1][j]), self.realToPixelPos(self.carHistPoints[i][j]), 1)
    
    ## UI and debug
    def drawMouseCone(self, drawPossibleConnections=True, drawConnectionThresholdCircle=False):
        """(UI element) show where you're about to place a cone and/or show the avaiable connections to new/hovered-over cone"""
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
        """debugging utility, allows certain debugging elements to be visualized (not carCam friendly)"""
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
        """(UI element) if active (button press), 'drag' the screen around by using the mouse"""
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
        """draw all map elements"""
        self.updateViewOffset() #handle mouse dragging
        self.background()
        self.drawFPScounter()
        self.drawCones(True) #boolean parameter is whether to draw lines between connected cones (track bounds) or not
        self.drawPathLines(True, self.drawTargetConeLines) #boolean parameters are whether to draw the lines between cones (not the line the car follows) and whether to draw circles (conesized ellipses) on the center points of path lines respectively
        self.drawFinishLine()
        self.drawCar()
        #debug and UI
        self.drawMouseCone(True, False)
        self.drawDebugLines()
    
    def updateWindowSize(self, drawSize=[1200, 600], drawOffset=[0,0], sizeScale=-1, autoMatchSizeScale=True):
        """handle the size of the window changing
            (optional) scale sizeScale (zooming) to match previous window size"""
        if(sizeScale > 0):
            self.sizeScale = sizeScale
        elif(autoMatchSizeScale):
            self.sizeScale = min(drawSize[0]/self.drawSize[0], drawSize[1]/self.drawSize[1]) * self.sizeScale #auto update sizeScale to match previous size
        self.drawSize = (int(drawSize[0]), int(drawSize[1]))
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1]))


##remote connection UI:
def remoteInstructionSend(socketToSendFrom, instruction):
    """send instruction to remove instance (car/host)"""
    if(socketToSendFrom.runningOnThread is not None):
        #print("threaded send")
        socketToSendFrom.manualSendBuffer.append(instruction)
    else:
        #print("manual send")
        try:
            socketToSendFrom.manualSend([instruction])
        except Exception as excep:
            print("remoteInstructionSend manualSend exception:", excep)

##these functions could probably be removed later (just put their contents in place of the function call)
def remoteConePlace(socketToSendFrom, coneToPlace, immediateConnect=False): #note: coneToPlace can either be a cone object or a tuple with arguments for Map.addCone()
    """instruct remote instance to place a cone"""
    #print("remoteConePlace")
    remoteInstructionSend(socketToSendFrom, ['PLACE', coneToPlace, immediateConnect])

def remoteConeConnect(socketToSendFrom, coneToConnect):
    """instruct remote instance to attempt to connect existing cone"""
    #print("remoteConeConnect")
    remoteInstructionSend(socketToSendFrom, ['CONNEC', coneToConnect])

def remoteConeSetFinish(socketToSendFrom, coneToUpdate): #could be changed to general 'coneUpdate' or 'coneEdit', if needed
    """instruct remote instance to set an existing cone as finish-line-cone"""
    #print("remoteConeSetFinish")
    remoteInstructionSend(socketToSendFrom, ['SETFIN', coneToUpdate])

def remoteConeDelete(socketToSendFrom, coneToDelete):
    """instruct remote instance to delete an existing cone"""
    #print("remoteConeDelete")
    remoteInstructionSend(socketToSendFrom, ['DELET', coneToDelete])

def remotePathFind(socketToSendFrom, numberOfPathPointsToFind):
    """instruct remote instance to try to generate a path (manual instruction)"""
    #print("remotePathFind")
    remoteInstructionSend(socketToSendFrom, ['PATH', numberOfPathPointsToFind])

def remoteAutoDrivingUpdate(socketToSendFrom, autoMode, targetSpeed):
    """instruct remote instance to update autodriving variables (.auto, .target_velocity)"""
    #print("remoteAutoDrivingUpdate")
    remoteInstructionSend(socketToSendFrom, ['AUTO', autoMode, targetSpeed])

def remoteAdjustMapSendInterval(socketToSendFrom, newMapSendInterval):
    """instruct remote instance to send more/less map packets"""
    #print("remoteAdjustMapSendInterval")
    remoteInstructionSend(socketToSendFrom, ['FPSADJ', int(newMapSendInterval)])

def remoteSaveMap(socketToSendFrom, filename=None):
    """instruct remote instance to save a (pandas) excel file
        the remote instance should respond with the file it saved (and the filename)"""
    #print("remoteSaveMap")
    remoteInstructionSend(socketToSendFrom, ['MAPSAV', filename])

def remoteWholeMapFileLoad(socketToSendFrom, map_file): #for debugging/testing, when you need to load an entire map object over the network
    """send & load a (pandas) excel file to remote instance"""
    #print("remoteWholeMapFileLoad")
    remoteInstructionSend(socketToSendFrom, ['MAPLOD', map_file])

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


def pygameInit(resolution): #you must specify a resolution
    """initialize pygame window
        (one pygame window can host multiple pygameDrawer objects by using the drawOffset variable)"""
    pygame.init()
    pygame.font.init()
    global window, oldWindowSize
    window = pygame.display.set_mode(resolution, pygame.RESIZABLE)
    oldWindowSize = window.get_size()
    pygame.display.set_caption("(pygame) selfdriving sim")
    global windowKeepRunning, windowStarted
    windowStarted = True
    windowKeepRunning = True

def pygameEnd():
    """deinitialize the pygame window (required for ending without crashing)"""
    global windowStarted
    if(windowStarted): #if the window never started, quit might error out or something stupid
        print("quitting pygame window...")
        pygame.quit()

def frameRefresh():
    """push the drawn frame(buffer) to the display"""
    pygame.display.flip() #send (finished) frame to display

def handleMousePress(pygamesimInput, buttonDown, button, pos, eventToHandle):
    """(UI element) handle the mouse-press-events"""
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
                    if((overlappingCone.ID == target.coneConData.cones[overlappingCone.LorR].ID) if (target.coneConData is not None) else False): #if the only connected cone is ALSO (already) in the target_list, then you cant make any more path
                        print("can't delete cone, it's in pathList") #just a lot easier, not impossible
                        deleting = False
                if(deleting):
                    print("deleting cone:", overlappingCone.ID)
                    if(pygamesimInput.isRemote):
                        remoteConeDelete(pygamesimInput.remoteUIsender, overlappingCone)
                    else:
                        pygamesimInput.mapToDraw.removeConeObj(overlappingCone)
            if(pygamesimInput.pathPlanningPresent):
                pygamesimInput.makeBoundrySplines()
        else:
            if((len(pygamesimInput.mapToDraw.finish_line_cones) < 2) if pygame.key.get_pressed()[pygame.K_f] else True):
                posToPlace = pygamesimInput.pixelsToRealPos(pos)
                overlaps, overlappingCone = pygamesimInput.mapToDraw.overlapConeCheck(posToPlace)
                if(overlaps):
                    if(pygame.key.get_pressed()[pygame.K_f]):
                        if((pygamesimInput.mapToDraw.finish_line_cones[0].LorR != overlappingCone.LorR) if (len(pygamesimInput.mapToDraw.finish_line_cones) > 0) else True):
                            if(pygamesimInput.isRemote):
                                remoteConeSetFinish(pygamesimInput.remoteUIsender, overlappingCone)
                            else:
                                overlappingCone.isFinish = True
                                pygamesimInput.mapToDraw.finish_line_cones.append(overlappingCone)
                        else:
                            print("can't set (existing) cone as finish, there's aready a "+("right" if leftOrRight else "left")+"-sided finish cone")
                    elif(pygamesimInput.coneConnecterPresent or pygamesimInput.isRemote):
                        if(pygamesimInput.isRemote):
                            remoteConeConnect(pygamesimInput.remoteUIsender, overlappingCone)
                        else:
                            pygamesimInput.mapToDraw.connectCone(overlappingCone)
                else:
                    # newConeID = GF.findMaxAttrIndex((pygamesimInput.mapToDraw.right_cone_list + pygamesimInput.mapToDraw.left_cone_list), 'ID')[1]
                    # aNewCone = Map.Cone(newConeID+1, posToPlace, leftOrRight, bool(pygame.key.get_pressed()[pygame.K_f]))
                    if(((pygamesimInput.mapToDraw.finish_line_cones[0].LorR != leftOrRight) if (len(pygamesimInput.mapToDraw.finish_line_cones) > 0) else True) if pygame.key.get_pressed()[pygame.K_f] else True):
                        if(pygamesimInput.isRemote):
                            #remoteConePlace(pygamesimInput.remoteUIsender, aNewCone, pygame.key.get_pressed()[pygame.K_LSHIFT])
                            remoteConePlace(pygamesimInput.remoteUIsender, (posToPlace, leftOrRight, bool(pygame.key.get_pressed()[pygame.K_f])), pygame.key.get_pressed()[pygame.K_LSHIFT])
                        else:
                            conePlaceSuccess, coneInList = pygamesimInput.mapToDraw.addCone(posToPlace, leftOrRight, bool(pygame.key.get_pressed()[pygame.K_f]))
                            if(pygame.key.get_pressed()[pygame.K_LSHIFT] and pygamesimInput.coneConnecterPresent):
                                pygamesimInput.mapToDraw.connectCone(coneInList)
                if(pygamesimInput.pathPlanningPresent):
                    pygamesimInput.makeBoundrySplines()
        if(pygame.key.get_pressed()[pygame.K_f]): #flag cursor stuff
            pygame.mouse.set_cursor(flagCurs24Data[0], flagCurs24Data[1], flagCurs24Data[2], flagCurs24Data[3]) #smaller flag cursor
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
            if(pygamesimInput.pathFinderPresent or pygamesimInput.isRemote):
                if(pygamesimInput.isRemote):
                    #remotePathFind(pygamesimInput.remoteUIsender, 1) #find 1 path point
                    remotePathFind(pygamesimInput.remoteUIsender, -1) #find as many path points as it can
                else:
                    #pygamesimInput.mapToDraw.makePath() #find a single path point
                    limitCounter = 0
                    while(pygamesimInput.mapToDraw.makePath() and (limitCounter<25)): #stops when path can no longer be advanced
                        limitCounter += 1
            if(pygamesimInput.pathPlanningPresent):
                pygamesimInput.makePathSpline()
        elif((key==pygame.K_a) or ((key==pygame.K_PLUS) or (key==pygame.K_EQUALS)) or (key==pygame.K_MINUS)):
            if(key==pygame.K_a): # a
                if((pygamesimInput.mapToDraw.car.pathFolData is not None) and pygamesimInput.pathPlanningPresent):
                    pygamesimInput.mapToDraw.car.pathFolData.auto = not pygamesimInput.mapToDraw.car.pathFolData.auto
                    if(not pygamesimInput.isRemote):
                        pygamesimInput.mapToDraw.car.desired_velocity = 0.0
                        pygamesimInput.mapToDraw.car.desired_steering = 0.0
                        try:
                            pygamesimInput.mapToDraw.car.sendSpeedAngle(pygamesimInput.mapToDraw.car.desired_velocity, pygamesimInput.mapToDraw.car.desired_steering)
                        except:
                            print("couldn't send stopping insctruction")
            elif((key==pygame.K_PLUS) or (key==pygame.K_EQUALS)): # +
                if((pygamesimInput.mapToDraw.car.pathFolData is not None) and pygamesimInput.pathPlanningPresent):
                    pygamesimInput.mapToDraw.car.pathFolData.targetVelocity += 0.25
            elif(key==pygame.K_MINUS): # -
                if((pygamesimInput.mapToDraw.car.pathFolData is not None) and pygamesimInput.pathPlanningPresent):
                    pygamesimInput.mapToDraw.car.pathFolData.targetVelocity -= 0.25
                    if(pygamesimInput.mapToDraw.car.pathFolData.targetVelocity < 0):
                        pygamesimInput.mapToDraw.car.pathFolData.targetVelocity = 0
            if(pygamesimInput.isRemote):
                remoteAutoDrivingUpdate(pygamesimInput.remoteUIsender, pygamesimInput.mapToDraw.car.pathFolData.auto, pygamesimInput.mapToDraw.car.pathFolData.targetVelocity)
        elif(key==pygame.K_q): # q (cubic splines sounds like 'q'-bic, also i suck at spelling
            pygamesimInput.drawQubicSplines = not pygamesimInput.drawQubicSplines #only has (the desired) effect if pyagmesimInput.pathPlanningPresent == True
        elif(key==pygame.K_t): # t
            pygamesimInput.drawTargetConeLines = not pygamesimInput.drawTargetConeLines #only has an effect if pyagmesimInput.pathFinderPresent == True
        elif(key==pygame.K_h): # h
            pygamesimInput.headlights = not pygamesimInput.headlights #only has an effect if car sprite is used (.carPolygonMode)
        elif(key==pygame.K_c): # c
            # if(pygame.key.get_pressed()[pygame.K_LCTRL]): #causes too many problems, just restart the program or run pygamesimInput.__init__ again
            #     pygamesimInput.mapToDraw.left_cone_list.clear()
            #     pygamesimInput.mapToDraw.right_cone_list.clear()
            #     pygamesimInput.mapToDraw.finish_line_cones.clear()
            #     pygamesimInput.mapToDraw.target_list.clear()
            #     pygamesimInput.mapToDraw.targets_full_circle = False
            #     #pygamesimInput.mapToDraw.clockStart = time.time()
                
            #     ## needlessly complicated, but (currently) function way of resetting object, which ONLY WORKS IF the __init__() argument names are the same as the class attribute names they go into
            #     initArgNames = getattr(getattr(getattr(pygamesimInput.mapToDraw.car, '__init__'), '__code__'), 'co_varnames')
            #     classAttrNames = dir(pygamesimInput.mapToDraw.car)
            #     argList = []
            #     for name in initArgNames:
            #         if(name in classAttrNames):
            #             argList.append(getattr(pygamesimInput.mapToDraw.car, name))
            #     argTuple = tuple(argList)
            #     pygamesimInput.mapToDraw.car = pygamesimInput.mapToDraw.car.__class__(*argTuple)
                
            #     if(pygamesimInput.mapToDraw.car.pathFolData is not None):
            #         pygamesimInput.mapToDraw.car.pathFolData = pygamesimInput.mapToDraw.car.pathFolData.__class__() #reset class
                
            #     if(pygamesimInput.pathPlanningPresent):
            #         pygamesimInput.left_spline = [[], []]
            #         pygamesimInput.right_spline = [[], []]
            #         pygamesimInput.path_midpoints_spline = [[], []]
            pygamesimInput.carHistPoints = []
            pygamesimInput.carHistTimer = pygamesimInput.mapToDraw.clock() #reset timer
        elif(key==pygame.K_v): # v
            pygamesimInput.carCam = not pygamesimInput.carCam
            if(pygamesimInput.carCam and pygamesimInput.movingViewOffset): #if you switched to carCam while you were moving viewOffset, just stop moving viewOffset (same as letting go of MMB)
                pygame.event.set_grab(0)
                pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
                pygamesimInput.updateViewOffset() #update it one last time (or at all, if this hasn't been running in redraw())
                pygamesimInput.movingViewOffset = False
        elif(key==pygame.K_RIGHTBRACKET):
            if(pygamesimInput.isRemote):
                pygamesimInput.remoteFPS += 1
                remoteAdjustMapSendInterval(pygamesimInput.remoteUIsender, int(pygamesimInput.remoteFPS))
        elif(key==pygame.K_LEFTBRACKET):
            if(pygamesimInput.isRemote):
                pygamesimInput.remoteFPS -= 1
                if(pygamesimInput.remoteFPS <= 0):
                    pygamesimInput.remoteFPS = 1
                remoteAdjustMapSendInterval(pygamesimInput.remoteUIsender, int(pygamesimInput.remoteFPS))
        elif(key==pygame.K_s): # s
            if(pygamesimInput.isRemote):
                remoteSaveMap(pygamesimInput.remoteUIsender)
            else:
                try:
                    saveStartTime = time.time()
                    pygamesimInput.save_map(pygamesimInput.mapToDraw) #currently, file name is auto-generated, because requesting UI text field input is a lot of effort, and python input() is blocking
                    print("save_map took", round(time.time()-saveStartTime, 2), "seconds")
                except Exception as excep:
                    print("failed to save file, exception:", excep)

def currentPygamesimInput(pygamesimInputList, mousePos=None, demandMouseFocus=True): #if no pos is specified, retrieve it using get_pos()
    """(UI element) return the pygameDrawer that the mouse is hovering over, or the one you interacted with last"""
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
    """(UI element) handle general (pygame) window-event"""
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
    
    elif(eventToHandle.type == pygame.DROPFILE): #drag and drop files to import them
        pygamesimInput = currentPygamesimInput(pygamesimInputList, None, False)
        print("attempting to load drag-dropped file:", eventToHandle.file)
        if(pygamesimInput.isRemote):
            try:
                import pandas as pd
                print("sending map_file to remote client...")
                remoteWholeMapFileLoad(pygamesimInput.remoteUIsender, pd.read_excel(eventToHandle.file, index_col=0)) #send pandas-dataframe to remote instance
            except Exception as excep:
                print("failed to send map_file to remote instance, exception:", excep)
        else:
            try:
                pygamesimInput.load_map(eventToHandle.file, pygamesimInput) #note: drag and drop functionality is a little iffy for multisim applications
                print("loaded file successfully")
            except Exception as excep:
                print("failed to load drag-dropped file, exception:", excep)
    
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

def handleAllWindowEvents(pygamesimInput): #input can be pygamesim object, 1D list of pygamesim objects or 2D list of pygamesim objects
    """(UI element) loop through (pygame) window-events and handle all of them"""
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
        if(eventToHandle.type != pygame.MOUSEMOTION): #skip mousemotion events early (fast)
            handleWindowEvent(pygamesimInputList, eventToHandle)
    
    # #the manual keyboard driving (tacked on here, because doing it with the event system would require more variables, and this is temporary anyway)
    # simToDrive = currentPygamesimInput(pygamesimInputList, demandMouseFocus=False) #get the active sim within the window
    # carToDrive = simToDrive.car
    # if((not carToDrive.pathFolData.auto) if simToDrive.pathPlanningPresent else True):
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







