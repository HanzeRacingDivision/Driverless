import pygame       #python game library, used for the visualization
import numpy as np  #general math library
import time         #used for FPS counter and stat text

import os #only used for loading the car sprite (os is used to get the filepath)
from PIL import Image, ImageDraw #python image library, only used for drawing car headlights

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use



class pygameDrawerCommon():
    """a class to render Map objects overtop of a camera stream"""
    def __init__(self, mapToDraw, window, drawSize=(1280,720), drawOffset=(0,0)):
        self.window = window #pass on the window object (pygame)
        self.drawSize = (int(drawSize[0]),int(drawSize[1])) #width and height of the display area (does not have to be 100% of the window)
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1])) #draw position offset, (0,0) is topleft
        
        self.mapToDraw = mapToDraw
        
        self.finishLineColor = [255,40,0] #red
        self.finishLineWidth = 2 #pixels wide
        
        self.leftConeColor = [255,255,0] #yellow
        self.rightConeColor = [0,50,255] #dark blue
        self.coneConnectionLineWidth = 2 #pixels wide

        self.leftUndetectedConeColor = [127,127,0] #faded yellow
        self.rightUndetectedConeColor = [0,25,127] #faded blue
        self.undetectedFinishLineColor = [127, 20, 0] #faded red
        
        self.pathColor = [0,220,255] #light blue
        self.pathLineWidth = 2 #pixels wide
        
        self.fontSize = 25
        self.fontColor = [200, 200, 200]
        self.pygameFont = pygame.font.Font(None, self.fontSize)
        
        self.FPStimer = time.time()
        self.FPSdata = []
        self.FPSdisplayInterval = 0.25
        self.FPSdisplayTimer = time.time()
        self.FPSrenderedFonts = []
        
        self.statDisplayTimer = time.time()
        self.statDisplayInterval = 0.1
        self.statRenderedFonts = []

        self.lastMapFilename = "" # the name of the loaded mapfile
        
        self.drawTargetConeLines = False #just for UI purposes, to toggle between showing and not showing how the targets are made
        self.drawConeSlamData = 0 #just for UI purposes, to toggle between showing lidar cone spot-count (and the actual datapoints) or not (neither)
        self.drawCarHist = False #just for UI purposes, to toggle between showing the position history (thin white line) or not
        self.extraViewMode = False #triggered with CTRL+V, can be used to switch between normal and 3D view, or whatever else you want
    
    def isInsideWindowPixels(self, pixelPos):
        """whether or not a pixel-position is inside the window"""
        return((pixelPos[0] < (self.drawSize[0] + self.drawOffset[0])) and (pixelPos[0] > self.drawOffset[0]) and (pixelPos[1] < (self.drawSize[1] + self.drawOffset[1])) and (pixelPos[1] > self.drawOffset[1]))
    
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
                self.FPSrenderedFonts.append(self.pygameFont.render(FPSstr, False, self.fontColor)) #render string (only 1 line per render allowed), no antialiasing, text color opposite of bgColor, background = bgColor
        for i in range(len(self.FPSrenderedFonts)):
            self.window.blit(self.FPSrenderedFonts[i], [self.drawOffset[0]+ self.drawSize[0]-5-self.FPSrenderedFonts[i].get_width(),self.drawOffset[1]+5+(i*self.fontSize)])
    
    def drawStatText(self):
        """draw some usefull information/statistics on-screen"""
        newTime = time.time()
        if((newTime - self.statDisplayTimer)>self.statDisplayInterval):
            self.statDisplayTimer = newTime
            statsToShow = [] # a list of strings
            carToDraw = self.mapToDraw.car
            statsToShow.append(str(round(carToDraw.position[0], 2))+" , "+str(round(carToDraw.position[1], 2))+" pos")
            statsToShow.append(str(round(np.rad2deg(carToDraw.angle), 2))+" = "+str(round(GF.degRoll(np.rad2deg(carToDraw.angle)), 2))+" angle")
            statsToShow.append(str(round(carToDraw.velocity, 2))+" & "+str(round(np.rad2deg(carToDraw.steering), 2))+" speed & steering")
            statsToShow.append(str(round(carToDraw.desired_velocity, 2))+" & "+str(round(np.rad2deg(carToDraw.desired_steering), 2))+" desired spd&str")
            if(self.mapToDraw.simVars is not None): #if we're simulating positional drift
                if(self.mapToDraw.simVars.car is not None):
                    statsToShow.append(str(round(self.mapToDraw.simVars.car.position[0], 2))+" , "+str(round(self.mapToDraw.simVars.car.position[1], 2))+" real sim pos")
                    statsToShow.append(str(round(np.rad2deg(self.mapToDraw.simVars.car.angle), 2))+" = "+str(round(GF.degRoll(np.rad2deg(self.mapToDraw.simVars.car.angle)), 2))+" real sim angle")
                if(self.mapToDraw.simVars.undiscoveredCones):
                    statsToShow.append("undiscoveredCones==True")
            try:
                debugMousePos = self.pixelsToRealPos(pygame.mouse.get_pos())
                statsToShow.append(str(round(debugMousePos[0], 2))+" , "+str(round(debugMousePos[1], 2))+" mouse pos")
            except:
                doNothing = 0
            statsToShow.append(str(carToDraw.pathFolData.auto)+"   "+str(carToDraw.pathFolData.laps))
            self.statRenderedFonts = [] # a list of rendered fonts (images)
            for textStr in statsToShow:
                self.statRenderedFonts.append(self.pygameFont.render(textStr, False, self.fontColor))
        for i in range(len(self.statRenderedFonts)):
            self.window.blit(self.statRenderedFonts[i], [self.drawOffset[0]+5,self.drawOffset[1]+5+(i*self.fontSize)])
    
    def drawLoadedFilename(self):
        if(len(self.lastMapFilename) > 0):
            renderedFont = self.pygameFont.render(self.lastMapFilename, False, self.fontColor)
            self.window.blit(renderedFont, [self.drawOffset[0]+self.drawSize[0]-renderedFont.get_width()-5,self.drawOffset[1]+self.drawSize[1]-renderedFont.get_height()-5])


class pygameDrawer(pygameDrawerCommon):
    """a class to draw Map objects (also includes drawing of UI some elements for manually making tracks)"""
    def __init__(self, mapToDraw, window, drawSize=(1200,600), drawOffset=(0,0), carCamOrient=0, sizeScale=100, startWithCarCam=False, invertYaxis=True):
        pygameDrawerCommon.__init__(self, mapToDraw, window, drawSize, drawOffset)
        
        self.viewOffset = [0.0, 0.0] #'camera' view offsets, changing this affects the real part of realToPixelPos()
        self.carCamOrient = carCamOrient #orientation of the car (and therefore everything) on the screen. 0 is towards the right
        self.sizeScale = sizeScale #pixels per meter
        self.carCam = startWithCarCam #it's either carCam (car-centered cam, with rotating but no viewOffset), or regular cam (with viewOffset, but no rotating)
        self.invertYaxis = invertYaxis #pygame has pixel(0,0) in the topleft, so this just flips the y-axis when drawing things
        
        self.bgColor = [50,50,50] #grey
        
        #self.pathCenterPixelDiam = 
        self.pathPointDiam = Map.Cone.coneDiam / 2
        
        self.splinePointDiam = Map.Cone.coneDiam / 3
        
        self.mouseCone = None #either None, True or False, to indicate the color of the (mouse) cone that is about to be placed (replaces floatingCone)
        
        #the drawing stuff:
        self.carColor = [50,200,50]
        #polygon stuff (to be replaced by sprite?)
        self.carPointRadius = None #will be calculated once the car is drawn for the first time
        self.carPointAngle = None #will be calculated once the car is drawn for the first time
        
        self.movingViewOffset = False
        self.prevViewOffset = (self.viewOffset[0], self.viewOffset[1])
        self.movingViewOffsetMouseStart = [0,0]
        
        # self.debugLines = [] #[[lineType, pos, pos/angles, color_index (0-2)], ] #third entry is: (lineType==0: straight line from two positions), (lineType==1: straight line from pos and [radius, angle]), (lineType==2: arc from pos and [radius, startAngle, endAngle])
        # self.debugLineColors = [[255,0,255],[255,255,255],[0,255,0], [255,160,255]] #purple, white, green, pink
        # self.debugLineWidth = 3
        
        ## load car sprite here (TBD)
        self.carPolygonMode = False #if no sprite is present, this can be used to draw a simple car
        try: #if the sprite doesnt load (usually because it's missing), thats fine
            current_dir = os.path.dirname(os.path.abspath(__file__)) #get directory name of current sketch
            self.car_image = pygame.image.load(os.path.join(current_dir, "carSprite.png"))
        except Exception as excep:
            print("couldn't load car sprite ", excep)
            print("using simple polygon car instead")
            self.carPolygonMode = True
        self.headlights = False #only has an effect if carPolygonMode=False, also doesnt really do anything right now
        
        self.drawCubicSplines = False
        
        # self.isRemote = False
        # self.remoteFPS = 5 #this should be initialised here though
        
        #self.drawCarHist = True
        self.carHistTimer = self.mapToDraw.clock()
        self.carHistPoints = []
        self.carHistTimeStep = 0.15
        self.carHistMinSquaredDistThresh = 0.1**2 #only save new positions if the car is moving
        self.carHistMaxLen = 200
        
        self.carKeyboardControlTimer = time.time() #ONLY USED FOR KEYBOARD DRIVING (commented out)
        
        try:
            self.viewOffset = [(-self.mapToDraw.car.position[0]) + ((self.drawSize[0]/self.sizeScale)/2), (-self.mapToDraw.car.position[1]) + ((self.drawSize[1]/self.sizeScale)/2)]
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
                return(np.array([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((self.drawSize[1]-pixelPos[1]+self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]]))
            else:
                return(np.array([((pixelPos[0]-self.drawOffset[0])/self.sizeScale)-self.viewOffset[0], ((pixelPos[1]-self.drawOffset[1])/self.sizeScale)-self.viewOffset[1]]))
    
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
    def isInsideWindowReal(self, realPos: np.ndarray):
        """whether or not a (real) position is inside the window (note: not computationally efficient)"""
        return(self.isInsideWindowPixels(self.realToPixelPos(realPos))) #not very efficient, but simple
    
    #drawing functions
    def background(self):
        """draw the background"""
        self.window.fill(self.bgColor, (self.drawOffset[0], self.drawOffset[1], self.drawSize[0], self.drawSize[1])) #dont fill entire screen, just this pygamesim's area (allowing for multiple sims in one window)
    
    def _dashedLine(self, lineColor, startPixelPos, endPixelPos, lineWidth, dashPixelPeriod=10, dashDutyCycle=0.5):
        pixelDist, angle = GF.distAngleBetwPos(startPixelPos, endPixelPos)
        for i in range(int(pixelDist/dashPixelPeriod)):
            dashStartPos = GF.distAnglePosToPos(i*dashPixelPeriod, angle, startPixelPos)
            dashEndPos = GF.distAnglePosToPos(i*dashPixelPeriod + dashPixelPeriod*dashDutyCycle, angle, startPixelPos)
            pygame.draw.line(self.window, lineColor, dashStartPos, dashEndPos, int(lineWidth))

    def _drawCubicSplinesList(self, splineListToDraw, splineColor, drawDots=True):
        splinePointPixelDiam = self.splinePointDiam * self.sizeScale
        for i in range(len(splineListToDraw[0])):
            splinePointPos = self.realToPixelPos([splineListToDraw[0][i], splineListToDraw[1][i]])
            if(drawDots):
                pygame.draw.ellipse(self.window, splineColor, [GF.ASA(-(splinePointPixelDiam/2), splinePointPos), [splinePointPixelDiam, splinePointPixelDiam]]) #draw cone
            if(i > 0):#if more than one spline point exists (and the forloop is past the first one)
                lastSplinePointPos = self.realToPixelPos([splineListToDraw[0][i-1], splineListToDraw[1][i-1]])
                pygame.draw.line(self.window, splineColor, lastSplinePointPos, splinePointPos, self.coneConnectionLineWidth) #line from center pos to center pos

    def _drawConeList(self, coneListToDraw, coneColor, drawLines=True, drawSlamData=0):
        conePixelDiam = Map.Cone.coneDiam * self.sizeScale
        drawnLineList = [] #[ [ID, ID], ] just a list of drawn lines by ID
        for cone in coneListToDraw:
            conePos = self.realToPixelPos(cone.position) #convert to pixel position
            ## draw the lines between cones
            if(drawLines):
                alreadyDrawn = [False, False]
                for drawnLine in drawnLineList:
                    for i in range(len(cone.connections)):
                        if((cone.ID in drawnLine) and (cone.connections[i].ID in drawnLine)):
                            alreadyDrawn[i] = True
                for i in range(len(cone.connections)):
                    if(not alreadyDrawn[i]):
                        pygame.draw.line(self.window, coneColor, conePos, self.realToPixelPos(cone.connections[i].position), self.coneConnectionLineWidth)
                        drawnLineList.append([cone.ID, cone.connections[i].ID]) #put established 'back' connection in list of drawn 
            ## now draw the cone itself
            #pygame.draw.circle(self.window, coneColor, [int(conePos[0]), int(conePos[1])], int(conePixelDiam/2)) #draw cone (as filled circle, not ellipse)
            coneEllipsePos = GF.ASA(-(conePixelDiam/2), conePos) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
            pygame.draw.ellipse(self.window, coneColor, [coneEllipsePos, [conePixelDiam, conePixelDiam]]) #draw cone
            
            if(drawSlamData > 0):
                if(cone.slamData is not None): # if the cone has a coneSlamData object
                    invConeColor = [255-coneColor[0], 255-coneColor[1], 255-coneColor[2]]
                    
                    trueConePos = cone.slamData.positions[0] #used to check if simulation is working correctly   ARC_TODO use simvars for this? maybe just remove this altogether
                    pygame.draw.ellipse(self.window, invConeColor, [GF.ASA(-(conePixelDiam/6), self.realToPixelPos(trueConePos)), [conePixelDiam/3, conePixelDiam/3]]) #draw cone
                    
                    try: #the blob does not HAVE to be stored in slamData (consider the extra pickling time), so if i end up removing it, please also uncomment this
                        if(drawSlamData > 1):
                            blob = cone.slamData.blobs[-1]
                            if(blob is not None):
                                adjustedConeDiam = Map.Cone.coneLidarDiam #TBD: calculate the diamter of the cone AT THE HEIGHT OF THE LIDAR (this does not have to be done dynamically, it can be constant)
                                for i in range(blob['pointCount']-1):
                                    superAdjustedConeRadius = np.cos(np.arcsin((blob['lines'][i][0]/2) / (adjustedConeDiam/2))) * (adjustedConeDiam/2)
                                    pygame.draw.line(self.window, invConeColor, self.realToPixelPos(blob['points'][i]), self.realToPixelPos(blob['points'][i+1]), self.coneConnectionLineWidth)
                                    blobLineCenter = GF.distAnglePosToPos(blob['lines'][i][0]/2, blob['lines'][i][1], blob['points'][i])
                                    blobLinePerpPos = GF.distAnglePosToPos(superAdjustedConeRadius, blob['lines'][i][1]+(np.pi/2), blobLineCenter)
                                    pygame.draw.line(self.window, invConeColor, self.realToPixelPos(blobLineCenter), self.realToPixelPos(blobLinePerpPos), self.coneConnectionLineWidth)
                    except:
                        doNothing = 0
                    
                    textStr = str(cone.slamData.counter) #get total-spotted-count
                    renderedText = self.pygameFont.render(textStr, False, invConeColor, coneColor)
                    coneTextPos = [conePos[0] - renderedText.get_size()[0]/2, conePos[1] - renderedText.get_size()[1]/2] #to topleft corner of text size
                    self.window.blit(renderedText, coneTextPos)
            else:
                textStr = str(cone.ID) #get total-spotted-count
                renderedText = self.pygameFont.render(textStr, False, [255-coneColor[0], 255-coneColor[1], 255-coneColor[2]], coneColor)
                coneTextPos = [conePos[0] - renderedText.get_size()[0]/2, conePos[1] - renderedText.get_size()[1]/2] #to topleft corner of text size
                self.window.blit(renderedText, coneTextPos)

    def drawCones(self, drawLines=True, drawSlamData=0):
        """draw the cones and their connections
            (if enabled, draw qubic splines instead of direct connections)"""
        if(self.mapToDraw.simVars is not None):
            for LorR in range(2):
                coneColor = self.rightUndetectedConeColor if LorR else self.leftUndetectedConeColor
                coneListToDraw = (self.mapToDraw.simVars.right_cone_list if LorR else self.mapToDraw.simVars.left_cone_list)
                self._drawConeList(coneListToDraw, coneColor, drawLines, 0)
        for LorR in range(2):
            coneColor = self.rightConeColor if LorR else self.leftConeColor
            coneListToDraw = (self.mapToDraw.right_cone_list if LorR else self.mapToDraw.left_cone_list)
            self._drawConeList(coneListToDraw, coneColor, drawLines and (not self.drawCubicSplines), drawSlamData)
            if(self.drawCubicSplines):
                # TBD: simVars splines?
                self._drawCubicSplinesList((self.mapToDraw.pathFolData.right_spline if LorR else self.mapToDraw.pathFolData.left_spline), coneColor)
                
    
    def drawPathLines(self, drawPoints=True, drawConeLines=False):
        """draw the path (target_list)
            (if enabled, draw qubic splines instead of direct connections)"""
        # target_list content:  [center point ([x,y]), [line angle, path (car) angle], track width, [ID, cone pos ([x,y]), index (left)], [(same as last entry but for right-side cone)], path-connection-strength]
        pathCenterPixelDiam = self.pathPointDiam * self.sizeScale
        if(self.drawCubicSplines):
            splinePointPixelDiam = self.splinePointDiam * self.sizeScale
            for i in range(len(self.mapToDraw.pathFolData.path_midpoints_spline[0])):
                targetPos = self.realToPixelPos([self.mapToDraw.pathFolData.path_midpoints_spline[0][i], self.mapToDraw.pathFolData.path_midpoints_spline[1][i]])
                if(drawPoints):
                    pygame.draw.ellipse(self.window, self.pathColor, [GF.ASA(-(splinePointPixelDiam/2), targetPos), [splinePointPixelDiam, splinePointPixelDiam]]) #draw spline point
                if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                    lastTargetPos = self.realToPixelPos([self.mapToDraw.pathFolData.path_midpoints_spline[0][i-1], self.mapToDraw.pathFolData.path_midpoints_spline[1][i-1]])
                    pygame.draw.line(self.window, self.pathColor, lastTargetPos, targetPos, self.pathLineWidth) #line from center pos to center pos
            # if(self.mapToDraw.targets_full_circle and (len(self.mapToDraw.pathFolData.path_midpoints_spline[0]) > 1)): #note: this depends on mapToDraw, which path_midpoints_spline might not, be careful
            #     startingPos = self.realToPixelPos([self.mapToDraw.pathFolData.path_midpoints_spline[0][0], self.mapToDraw.pathFolData.path_midpoints_spline[1][0]])
            #     endingPos = self.realToPixelPos([self.mapToDraw.pathFolData.path_midpoints_spline[0][-1], self.mapToDraw.pathFolData.path_midpoints_spline[1][-1]])
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
        if((len(self.mapToDraw.simVars.finish_line_cones) >= 2) if (self.mapToDraw.simVars is not None) else False): # draw simVars finish line first (underneath the main one)
            pygame.draw.line(self.window, self.undetectedFinishLineColor, self.realToPixelPos(self.mapToDraw.simVars.finish_line_cones[0].position), self.realToPixelPos(self.mapToDraw.simVars.finish_line_cones[1].position), self.finishLineWidth)
        if(len(self.mapToDraw.finish_line_cones) >= 2):
            pygame.draw.line(self.window, self.finishLineColor, self.realToPixelPos(self.mapToDraw.finish_line_cones[0].position), self.realToPixelPos(self.mapToDraw.finish_line_cones[1].position), self.finishLineWidth)
        
    def drawCar(self, carToDrawOverwrite=None, carPolygonModeOverwrite=None, headlightsOverwrite=None, recursiveAllowed=True):
        """draw car sprite
            (or a simple polygon instead, if sprite failed/disabled)"""
        carToDraw = self.mapToDraw.car
        if(carToDrawOverwrite is not None):
            carToDraw = carToDrawOverwrite
        elif(((self.mapToDraw.simVars.car is not None) if (self.mapToDraw.simVars is not None) else False) and recursiveAllowed): #if this is a simulation (elif to make sure we dont do this recursively endlessly)      ARC_TODO improve this?
            self.drawCar(self.mapToDraw.simVars.car, True, False, False) # draw virtual car (without positionalDrift and SLAM and stuff) into 
        
        chassisCenter = carToDraw.getChassisCenterPos()
        if(carPolygonModeOverwrite if (carPolygonModeOverwrite is not None) else self.carPolygonMode):
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
            arrowPoints = [self.realToPixelPos(carToDraw.position), polygonPoints[1], polygonPoints[2]] 
            oppositeColor = [255-self.carColor[0], 255-self.carColor[1], 255-self.carColor[1]]
            pygame.draw.polygon(self.window, oppositeColor, arrowPoints) #draw arrow
        else:
            if(headlightsOverwrite if (headlightsOverwrite is not None) else self.headlights):# draw headlights (first, to not overlap car sprite)
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
            pygame.draw.circle(window, [255, 255, 255], self.realToPixelPos(carToDraw.position), 0.02 * self.sizeScale) #draw car position indicator
        
        if((carToDraw.pathFolData.nextTarget is not None) if (carToDraw.pathFolData is not None) else False):
            targetPos = self.realToPixelPos(carToDraw.pathFolData.nextTarget.position)
            carPos = self.realToPixelPos(carToDraw.position)
            pygame.draw.line(self.window, [255, 255, 255], carPos, targetPos, 1)
        
        if((self.mapToDraw.clock() - self.carHistTimer) > self.carHistTimeStep):
            self.carHistTimer = self.mapToDraw.clock()
            if((GF.distSqrdBetwPos(np.array(self.carHistPoints[-1][0]), carToDraw.position) > self.carHistMinSquaredDistThresh) if (len(self.carHistPoints) > 1) else True):
                self.carHistPoints.append([carToDraw.position]) #you can add any number of points to store, as long as the first one is carToDraw.position
                if(len(self.carHistPoints) > self.carHistMaxLen):
                    self.carHistPoints.pop(0)
        
        if((len(self.carHistPoints) > 1) and self.drawCarHist):
            for i in range(1, len(self.carHistPoints)):
                for j in range(len(self.carHistPoints[i])):
                    pygame.draw.line(self.window, [200, 200, 200], self.realToPixelPos(self.carHistPoints[i-1][j]), self.realToPixelPos(self.carHistPoints[i][j]), 1)
    
    ## UI and debug
    def _drawMouseCone(self, mapToUse, drawPossibleConnections, drawMaxConnectionDistCircle, isUndetected=False):
        """(UI element) show where you're about to place a cone and/or show the avaiable connections to new/hovered-over cone"""
        if(self.mouseCone is not None): #if there is a floating cone to be drawn
            conePixelDiam = Map.Cone.coneDiam * self.sizeScale
            conePixelPos = pygame.mouse.get_pos() #update position to match mouse position
            if(self.isInsideWindowPixels(conePixelPos)): #should always be true, right?
                coneColor = (self.rightUndetectedConeColor if self.mouseCone else self.leftUndetectedConeColor) if isUndetected else (self.rightConeColor if self.mouseCone else self.leftConeColor)
                overlapsCone, overlappingCone = mapToUse.overlapConeCheck(self.pixelsToRealPos(conePixelPos))
                if(overlapsCone):
                    coneColor = (self.rightUndetectedConeColor if overlappingCone.LorR else self.leftUndetectedConeColor) if isUndetected else (self.rightConeColor if overlappingCone.LorR else self.leftConeColor) #overlapping cone might have other cone color
                    conePixelPos = self.realToPixelPos(overlappingCone.position)
                else:
                    coneCornerPos = GF.ASA(-(conePixelDiam/2), conePixelPos) #bounding box of ellipse is positioned in topleft corner, so shift cone half a conesize to the topleft.
                    pygame.draw.ellipse(self.window, coneColor, [coneCornerPos, [conePixelDiam, conePixelDiam]]) #draw cone
                if(drawMaxConnectionDistCircle):
                    import coneConnecting as CC
                    pygame.draw.circle(self.window, coneColor, [int(conePixelPos[0]), int(conePixelPos[1])], CC.coneConnecter.maxConnectionDist * self.sizeScale, self.coneConnectionLineWidth) #draw circle with maxConnectionDist radius 
                if(drawPossibleConnections):
                    import coneConnecting as CC
                    nearbyConeList = [];   bestConnection = None # init vars
                    if(overlapsCone):
                        if(len(overlappingCone.connections) < 2):
                            nearbyConeList = mapToUse.distanceToConeSquared(overlappingCone.position, mapToUse.right_cone_list if overlappingCone.LorR else mapToUse.left_cone_list, False, [overlappingCone.ID], CC.coneConnecter.maxConnectionDistSquared, 'EXCL_DUBL_CONN', [overlappingCone.ID])
                            _, bestConnection = CC.connectCone(mapToUse, overlappingCone, applyResult=False, printDebug=False)
                    else:
                        nearbyConeList = mapToUse.distanceToConeSquared(self.pixelsToRealPos(conePixelPos), mapToUse.right_cone_list if self.mouseCone else mapToUse.left_cone_list, False, [], CC.coneConnecter.maxConnectionDistSquared, 'EXCL_DUBL_CONN', [])
                        tempCone = Map.Cone(pos=self.pixelsToRealPos(conePixelPos),leftOrRight=self.mouseCone) # a bit sketchy, but works..
                        _, bestConnection = CC.connectCone(mapToUse, tempCone, applyResult=False, printDebug=False)
                    dashedLinePixelPeriod = max(conePixelDiam * 0.5, min(*self.drawSize)/50)
                    for cone in nearbyConeList:
                        # pygame.draw.line(self.window, coneColor, conePixelPos, self.realToPixelPos(cone[0].position), int(self.coneConnectionLineWidth/2)) # draw a solid line
                        self._dashedLine(coneColor, conePixelPos, self.realToPixelPos(cone[0].position), int(self.coneConnectionLineWidth/2), dashedLinePixelPeriod) # draw a dashed line
                    if(bestConnection is not None):
                        pygame.draw.line(self.window, coneColor, conePixelPos, self.realToPixelPos(bestConnection.position), self.coneConnectionLineWidth) # draw a solid line

    def drawMouseCone(self, drawPossibleConnections=True, drawMaxConnectionDistCircle=True, drawSimvarLines=True):
        if(drawSimvarLines and (self.mapToDraw.simVars.undiscoveredCones if (self.mapToDraw.simVars is not None) else False)):
            self._drawMouseCone(self.mapToDraw.simVars, drawPossibleConnections, drawMaxConnectionDistCircle, isUndetected=True)
        else:
            self._drawMouseCone(self.mapToDraw, drawPossibleConnections, drawMaxConnectionDistCircle, isUndetected=False)
    
    # def drawDebugLines(self):
    #     """debugging utility, allows certain debugging elements to be visualized (not carCam friendly)"""
    #     #debugLines structure: [pos,pos, color_index (0-2)]
    #     for debugLine in self.debugLines:
    #         if(abs(debugLine[0]) == 2):
    #             if(debugLine[0] == 2):
    #                 pixelRactSize = debugLine[2][0] * self.sizeScale
    #                 debugLine[1] = GF.ASA(-(pixelRactSize), debugLine[1])
    #                 pixelRactSize *= 2
    #                 debugLine[2][0] = pixelRactSize
    #                 debugLine[0] = -2
    #             pygame.draw.arc(self.window, self.debugLineColors[(debugLine[3] if (len(debugLine)==4) else 0)], [debugLine[1], [debugLine[2][0], debugLine[2][0]]], debugLine[2][1], debugLine[2][2], self.debugLineWidth)
    #         else:
    #             if(debugLine[0] == 1):
    #                 secondPos = self.realToPixelPos(GF.distAnglePosToPos(debugLine[2][0], debugLine[2][1], self.pixelsToRealPos(debugLine[1]))) #convert
    #                 debugLine[2] = secondPos
    #                 debugLine[0] = -1
    #             pygame.draw.line(self.window, self.debugLineColors[(debugLine[3] if (len(debugLine)==4) else 0)], debugLine[1], debugLine[2], self.debugLineWidth)
    
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
        self.drawCones(True, self.drawConeSlamData) #boolean parameter is whether to draw lines between connected cones (track bounds) or not
        # if(self.drawCubicSplines):
        #     self.drawCubicSplines(True) # TODO: make seperate function
        self.drawPathLines(True, self.drawTargetConeLines) #boolean parameters are whether to draw the lines between cones (not the line the car follows) and whether to draw circles (conesized ellipses) on the center points of path lines respectively
        self.drawFinishLine()
        self.drawCar()
        self.drawStatText()
        self.drawLoadedFilename()
        #debug and UI
        self.drawMouseCone()
        #self.drawDebugLines()
    
    def updateWindowSize(self, drawSize=[1200, 600], drawOffset=[0,0], sizeScale=-1, autoMatchSizeScale=True):
        """handle the size of the window changing
            (optional) scale sizeScale (zooming) to match previous window size"""
        if(sizeScale > 0):
            self.sizeScale = sizeScale
        elif(autoMatchSizeScale):
            self.sizeScale = min(drawSize[0]/self.drawSize[0], drawSize[1]/self.drawSize[1]) * self.sizeScale #auto update sizeScale to match previous size
        self.drawSize = (int(drawSize[0]), int(drawSize[1]))
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1]))
        print("updateWindowSize:", self.drawSize, self.drawOffset, self.sizeScale, autoMatchSizeScale)



class pygameDrawer3D(pygameDrawerCommon):
    """a class to render Map objects overtop of a camera stream"""
    def __init__(self, mapToDraw, window, drawSize=(1280,720), drawOffset=(0,0)):
        pygameDrawerCommon.__init__(self, mapToDraw, window, drawSize, drawOffset)
        
        self.coneOutlineWidth = 5        #pixels wide
        
        self.pathCenterPixelDiam = 10    #pixel diameter
        
        self.flagImg = makeFlagImg((self.drawSize[0]/10, self.drawSize[1]/10), (6,4))
        self.flagImgSizeMult = 1.0 #just a size multiplier, change based on feeling
        
        # TBD
        self.cameraSurface = pygame.Surface(self.drawSize)
        # current_dir = os.path.dirname(os.path.abspath(__file__))                 #deleteme
        # self.cameraSurface = pygame.image.load(os.path.join(current_dir, "coneImg.png")) #deleteme
        
        # constants, TO BE MOVED TO Map.py (to cone- and car classes respectively)
        self.coneHeight = 0.4
        self.cameraHeight = 0.3
        self.cameraFOV = (np.deg2rad(62.2), np.deg2rad(48.8))
        # for exmplanation of math, see self.perspectiveProjection()
        #  this if for a projection plane at distance 1.0
        self.camProjPlaneMult = ((drawSize[0]/2) / (1.0*np.tan(self.cameraFOV[0]/2)),
                                 (drawSize[1]/2) / (1.0*np.tan(self.cameraFOV[1]/2))) # (Yres/2) / plane_height
        
        self.FOVmargin = np.deg2rad(5) #without this, cones will not be rendered even when 0.49 of them is already/still in frame
        self.renderAngleMax = min((self.cameraFOV[0]/2)+(self.FOVmargin*5), np.deg2rad(88)) #has to be less than pi/2 (90deg)
        self.renderDistMin = np.tan(min((self.cameraFOV[1]/2)+(self.FOVmargin*5), np.deg2rad(88)))*self.cameraHeight
        
        #self.lidarHeight = 0.2 #there are currently too many complications (multicore data unavailability) to show lidar data in here
        
    
    def realToRespectivePos(self, realPos: np.ndarray):
        """covert global coordinates to (2D) position with respect to the car"""
        dist, angle = GF.distAngleBetwPos(self.mapToDraw.car.position, realPos)
        return(GF.distAnglePosToPos(dist, angle-self.mapToDraw.car.angle, np.zeros(2)))
    
    def perspectiveProjection(self, realPos: np.ndarray, z=None, camHeightIsZeroZ=False):
        """convert respective-to-car (top-down) real coordinates to a camera (FPV) pixel coordinates
            note: the camera looks towards the 0 angle in top-down, so paralel to the X-axis"""
        #if((len(realPos) < 2) or (len(realPos) > 3)):
        #    print("misuse of perspectiveProjection, bad entry")
        #    return(np.zeros(2))
        if(realPos[0] < 0.0001): #avoid divide by 0 AND only allow coordinates in front of (not behind) the camera
            print("bad use of perspectiveProjection(), realPos is behind (or perfectly next to) the camera!")
            return(np.zeros(2))
        realPos3D = np.array([realPos[0], realPos[1], 0.0], dtype=np.float64)
        if(len(realPos) < 3): #if realPos is the 2D pos, and the manual Z parameter is used
            if(z is None):
                print("OH NO!, you forgot the 3rd axis entry in perspectiveProjection(). Assuming z=0.0")
            else:
                realPos3D[2] = z;
        else:
            realPos3D[2] = realPos[2] #if realPos was already 3D
        if(not camHeightIsZeroZ): #if the camera is not considered to be the center of the Z axis (but still the center of X and Y)
            realPos3D[2] -= self.cameraHeight
        # for perspective projection, one takes a limited plane (rectangle) with normal-vector N
        # then, given original point (vector) X, you can calculate a vector P that lies (ends) on the plane,
        #  by calculating P = lambda*X. (in other words, lambda is a length-multiplier which makes X end when it hits the plane)
        # To get the value of (scalar) lambda, you can do  lambda = d / (X . N) = distance between camera point (0,0,0) and plane,
        #  divided by the dot-product of X and N.
        # to make the math simpler, you choose an N that is 1 in one axis, and 0 in the other two axis,
        #  now any dot-product with N = 1*x + 0*y + 0*z = x
        # furthermore, if we take a distance-to-plane ('d') of 1, it becomes  lambda = 1 / (X . N) = 1/x
        #  which means lambda*X = [x/x, y/x, z/x] = [1, y/x, z/x]
        # the size of the plane is based of the FOV of the camera (and of course the distance ('d') to it)
        # normalizing the newfound coordinates to (0, 1.0) or (-0.5, 0.5)
        #  and then multiplying by the camera resolution can be combined into 1 action
        # for this we just calculate some constants earlier
        pixelPos = np.array([int((-realPos3D[1]/realPos3D[0]) * self.camProjPlaneMult[0]), #positive y -> negative rotated x
                             int((realPos3D[2]/realPos3D[0]) * self.camProjPlaneMult[1])])
        # do note: for this math, [x, 0, 0] is the middle of the screen
        pixelPos[0] += int(self.drawSize[0]/2)
        pixelPos[1] += int(self.drawSize[1]/2); pixelPos[1] = self.drawSize[1] - pixelPos[1] #also invert, because pixel(0,0) is topleft
        return(pixelPos)
    
    def realToPixelPos(self, realPos: np.ndarray, z, camHeightIsZeroZ=False):
        """given a real (2D) pos and a z-height, calculate pixel position on screen (or off-screen)
            i don't yet know what happens if the pos is wildy out of the FOV"""
        return(self.perspectiveProjection(self.realToRespectivePos(realPos), z, camHeightIsZeroZ))
    
    def distAngleToPixelPos(self, distAngle, z, camHeightIsZeroZ=False):
        return(self.perspectiveProjection(GF.distAnglePosToPos(distAngle[0], distAngle[1], np.zeros(2)), z, camHeightIsZeroZ))
    
    def updateCameraFrame(self, buffer, size, encoding):
        self.cameraSurface = pygame.image.frombuffer(buffer, size, encoding) #https://www.pygame.org/docs/ref/image.html#pygame.image.frombuffer
    
    def drawCameraFrame(self):
        """draw camera frame as background"""
        self.window.blit(self.cameraSurface, self.drawOffset)

    def drawCones(self, drawLines=True, fillCone=True, drawSlamData=0):
        """draw cones and their connections (closest cones drawn last)
            drawing connections is TBD (because i want to get the drawing order right)"""
        angleMargin = self.cameraFOV[0]/2 + self.FOVmargin
        nearbyConeList = self.mapToDraw.distanceToCone(self.mapToDraw.car.position, None, sortBySomething='SORTBY_DIST', 
                                                       angleThreshRange=[self.mapToDraw.car.angle - angleMargin, self.mapToDraw.car.angle + angleMargin])
        coneIDlist = []
        for i in range(len(nearbyConeList)): #first, calculate cone position with respect to the car
            nearbyConeList[i].append(GF.distAnglePosToPos(nearbyConeList[i][1][0], nearbyConeList[i][1][1]-self.mapToDraw.car.angle, np.zeros(2)))
            coneIDlist.append(nearbyConeList[i][0].ID)
        
        drawnLineList = [] #[ [ID, ID], ] just a list of drawn lines by ID
        for i in range(len(nearbyConeList)):
            cone, distAngle, respectivePos = nearbyConeList[len(nearbyConeList)-1-i] #scroll through array backwards, to render the closest cones last
            coneColor = self.rightConeColor if cone.LorR else self.leftConeColor
            
            if(drawLines):
                alreadyDrawn = [False, False]
                for drawnLine in drawnLineList:
                    for j in range(len(cone.connections)):
                        if((cone.ID in drawnLine) and (cone.connections[j].ID in drawnLine)):
                            alreadyDrawn[j] = True
                for j in range(len(cone.connections)):
                    if(not alreadyDrawn[j]):
                        ## only draw lines if that cone is also visible
                        # connectedConeVisible = False; equivalentEntry=-1
                        # for k in range(len(coneIDlist)):
                        #     if(coneIDlist[k] == cone.connections[j].ID):
                        #         connectedConeVisible = True;  equivalentEntry = k
                        # if(connectedConeVisible):
                        #     pygame.draw.line(self.window, coneColor, self.perspectiveProjection(respectivePos, 0.0), self.perspectiveProjection(nearbyConeList[equivalentEntry][2], 0.0), self.coneConnectionLineWidth)
                        
                        # draw lines if connected cone lies ahead of (not behind) car
                        dist, angle = GF.distAngleBetwPos(self.mapToDraw.car.position, cone.connections[j].position)
                        angle = GF.radRoll(angle-self.mapToDraw.car.angle)
                        if((abs(angle) < self.renderAngleMax) and (dist > self.renderDistMin)):
                            pygame.draw.line(self.window, coneColor, self.perspectiveProjection(respectivePos, 0.0), self.distAngleToPixelPos((dist, angle), 0.0), self.coneConnectionLineWidth)
                        
                        drawnLineList.append([cone.ID, cone.connections[j].ID]) #put established 'back' connection in list of drawn 
            # calculate cone (triangle) points
            perpAngle = distAngle[1] - self.mapToDraw.car.angle + np.pi/2 # perpendicular angle
            coneRadius = Map.Cone.coneDiam / 2
            coneBottomOffset = np.array([np.cos(perpAngle)*coneRadius, np.sin(perpAngle)*coneRadius])
            conePoints = [self.perspectiveProjection(respectivePos, self.coneHeight), #top point
                          self.perspectiveProjection(respectivePos+coneBottomOffset, 0.0), #bottom point 1
                          self.perspectiveProjection(respectivePos-coneBottomOffset, 0.0)] #bottom point 2
            # draw cone
            if(fillCone):
                pygame.draw.polygon(self.window, coneColor, conePoints)
            else:
                for i in range(len(conePoints)):
                    pygame.draw.line(self.window, coneColor, conePoints[i], conePoints[(i+1)%len(conePoints)], self.coneOutlineWidth)
            
            if(drawSlamData > 0): # ARC_TODO maybe the new lidar setup won't have this data available
                if(cone.slamData is not None): #pre-SLAM lidar cone spotting
                    textStr = str(cone.slamData.counter) #get total-spotted-count
                    renderedText = self.pygameFont.render(textStr, False, [255-coneColor[0], 255-coneColor[1], 255-coneColor[2]], coneColor)
                    coneTextPos = [conePoints[0][0], (conePoints[0][1]+conePoints[1][1])/2] #pixel position at (approx) center of cone
                    coneTextPos[0] -= renderedText.get_size()[0]/2;   coneTextPos[1] -= renderedText.get_size()[1]/2; #to topleft corner of text size
                    self.window.blit(renderedText, coneTextPos)
            
            # draw finish flag
            if(cone.isFinish):
                flagSizeMult = self.flagImgSizeMult / distAngle[0]
                scaledFlagImg = pygame.transform.scale(self.flagImg, (int(self.flagImg.get_width()*flagSizeMult), int(self.flagImg.get_height()*flagSizeMult)))
                self.window.blit(scaledFlagImg, (conePoints[0]-int(scaledFlagImg.get_width()/2), conePoints[0]-scaledFlagImg.get_height()))
    
    def drawPathLines(self, drawPoints=False, drawConeLines=False): #qubic splines TBD(?)
        """draw the path (target_list)"""
        for i in range(len(self.mapToDraw.target_list)):
            #if(self.isInsideWindowReal(self.mapToDraw.target_list[i].position)):
            if(drawPoints):
                pointDistAngle = GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.target_list[i].position)
                pointDistAngle[1] = GF.radRoll(pointDistAngle[1]-self.mapToDraw.car.angle)
                if((abs(pointDistAngle[1]) < self.renderAngleMax) and (pointDistAngle[0] > self.renderDistMin)): #if current point lies ahead of (not behind) car
                    pointPixelPos = self.distAngleToPixelPos(pointDistAngle, 0.0)
                    #pygame.draw.circle(self.window, self.pathColor, pointPixelPos, int(self.pathCenterPixelDiam/2)) #draw center point (as filled circle, not ellipse)
                    pygame.draw.ellipse(self.window, self.pathColor, [GF.ASA(-(self.pathCenterPixelDiam/2), pointPixelPos), [self.pathCenterPixelDiam, self.pathCenterPixelDiam]]) #draw center point
            if(drawConeLines and (self.mapToDraw.target_list[i].coneConData is not None)): #instead of checking if pathFinderPresent, we can just check if the data is there (also more isRemote friendly)
                coneDistAngles = [GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.target_list[i].coneConData.cones[0].position), GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.target_list[i].coneConData.cones[1].position)]
                coneDistAngles[0][1] = GF.radRoll(coneDistAngles[0][1]-self.mapToDraw.car.angle);     coneDistAngles[1][1] = GF.radRoll(coneDistAngles[1][1]-self.mapToDraw.car.angle)
                if((abs(coneDistAngles[0][1]) < self.renderAngleMax) and (coneDistAngles[0][0] > self.renderDistMin) and (abs(coneDistAngles[1][1]) < self.renderAngleMax) and (coneDistAngles[1][0] > self.renderDistMin)): #if both cones lie ahead of (not behind) car
                    pygame.draw.line(self.window, self.pathColor, self.distAngleToPixelPos(coneDistAngles[0], 0.0), self.distAngleToPixelPos(coneDistAngles[1], 0.0), self.pathLineWidth) #line from left cone to right cone
            if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                #draw line between center points of current pathline and previous pathline (to make a line that the car should (sort of) follow)
                if((abs(pointDistAngle[1]) < self.renderAngleMax) and (pointDistAngle[0] > self.renderDistMin)): #if current point lies ahead of (not behind) car
                    lastPointDistAngle = GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.target_list[i-1].position)
                    lastPointDistAngle[1] = GF.radRoll(lastPointDistAngle[1]-self.mapToDraw.car.angle)
                    if((abs(lastPointDistAngle[1]) < self.renderAngleMax) and (lastPointDistAngle[0] > self.renderDistMin)): #if last point lies ahead of (not behind) car
                        pygame.draw.line(self.window, self.pathColor, self.distAngleToPixelPos(lastPointDistAngle, 0.0), pointPixelPos, self.pathLineWidth) #line from center pos to center pos
        if(self.mapToDraw.targets_full_circle):
            endDistAngles = [GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.target_list[-1].position), GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.target_list[0].position)]
            endDistAngles[0][1] = GF.radRoll(endDistAngles[0][1]-self.mapToDraw.car.angle);     endDistAngles[1][1] = GF.radRoll(endDistAngles[1][1]-self.mapToDraw.car.angle)
            if((abs(endDistAngles[0][1]) < self.renderAngleMax) and (endDistAngles[0][0] > self.renderDistMin) and (abs(endDistAngles[1][1]) < self.renderAngleMax) and (endDistAngles[1][0] > self.renderDistMin)): #if both points lie ahead of (not behind) car
                pygame.draw.line(self.window, self.pathColor, self.distAngleToPixelPos(endDistAngles[0], 0.0), self.distAngleToPixelPos(endDistAngles[1], 0.0), self.pathLineWidth) #line that loops around to start
    
    def drawFinishLine(self, drawTopLine=True):
        """draw finish line (between finish_line_cones)"""
        if(len(self.mapToDraw.finish_line_cones) >= 2):
            coneDistAngles = [GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.finish_line_cones[0].position), GF.distAngleBetwPos(self.mapToDraw.car.position, self.mapToDraw.finish_line_cones[1].position)]
            coneDistAngles[0][1] = GF.radRoll(coneDistAngles[0][1]-self.mapToDraw.car.angle);     coneDistAngles[1][1] = GF.radRoll(coneDistAngles[1][1]-self.mapToDraw.car.angle)
            if((abs(coneDistAngles[0][1]) < self.renderAngleMax) and (coneDistAngles[0][0] > self.renderDistMin) and (abs(coneDistAngles[1][1]) < self.renderAngleMax) and (coneDistAngles[1][0] > self.renderDistMin)): #if both cones lie ahead of (not behind) car
                respectivePoses = [GF.distAnglePosToPos(coneDistAngles[0][0], coneDistAngles[0][1], np.zeros(2)), GF.distAnglePosToPos(coneDistAngles[1][0], coneDistAngles[1][1], np.zeros(2))]
                pygame.draw.line(self.window, self.finishLineColor, self.perspectiveProjection(respectivePoses[0], 0.0), self.perspectiveProjection(respectivePoses[1], 0.0), self.finishLineWidth)
                if(drawTopLine):
                    pygame.draw.line(self.window, self.finishLineColor, self.perspectiveProjection(respectivePoses[0], self.coneHeight), self.perspectiveProjection(respectivePoses[1], self.coneHeight), self.finishLineWidth)
    
    def redraw(self):
        """draw new frame"""
        self.drawCameraFrame()
        self.drawPathLines(True, self.drawTargetConeLines)
        self.drawFinishLine(True)
        self.drawCones(True, True, self.drawConeSlamData)
        self.drawFPScounter()
        self.drawStatText()
        self.drawLoadedFilename()



def makeFlagImg(size, checkerPattern=(6,6)):
    returnSurface = pygame.Surface(size)
    checkerSize = (size[0]/checkerPattern[0], size[1]/checkerPattern[1])
    cehckerSizeInt = (int(checkerSize[0]), int(checkerSize[1]))
    checkerColor = [255, 255, 255]
    for i in range(checkerPattern[0]):
        for j in range(checkerPattern[1]):
            if(((i+j) % 2) == 1):
                returnSurface.fill(checkerColor, ((i*checkerSize[0], j*checkerSize[1]), cehckerSizeInt))
    return(returnSurface)






global windowKeepRunning, windowStarted
windowStarted = False
windowKeepRunning = False

global oldWindowSize
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






