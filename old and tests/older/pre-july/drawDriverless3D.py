import pygame       #python game library, used for the visualization
import numpy as np  #general math library
import time         #used for FPS counter and stat text

import os

from Map import Map
import generalFunctions as GF



class pygameDrawer3D():
    """a class to render Map objects overtop of a camera stream"""
    def __init__(self, mapToDraw, window, drawSize=(1280,720), drawOffset=(0,0)):
        self.window = window #pass on the window object (pygame)
        self.drawSize = (int(drawSize[0]),int(drawSize[1])) #width and height of the display area (does not have to be 100% of the window)
        self.drawOffset = (int(drawOffset[0]), int(drawOffset[1])) #draw position offset, (0,0) is topleft
        
        self.mapToDraw = mapToDraw
        
        self.finishLineColor = [255,40,0]
        self.finishLineWidth = 2 #pixels wide
        
        self.leftConeColor = [255,255,0] #yellow
        self.rightConeColor = [0,50,255] #dark blue
        self.coneOutlineWidth = 5        #pixels wide
        self.coneConnectionLineWidth = 2 #pixels wide
        
        self.pathColor = [0,220,255] #light blue
        self.pathLineWidth = 2 #pixels wide
        self.pathCenterPixelDiam = 10
        
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
        
        self.flagImg = makeFlagImg((self.drawSize[0]/10, self.drawSize[1]/10), (6,4))
        self.flagImgSizeMult = 1.0 #just a size multiplier, change based on feeling
        
        # TBD
        #self.cameraSurface = pygame.surface(self.drawSize)
        current_dir = os.path.dirname(os.path.abspath(__file__))                 #deleteme
        self.cameraSurface = pygame.image.load(os.path.join(current_dir, "coneImg.png")) #deleteme
        
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
        dist, angle = GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), realPos)
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
        realPos3D = np.array([realPos[0], realPos[1], 0.0], dtype=np.float32)
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
    
    def cameraFrame(self):
        """draw camera frame as background"""
        self.window.blit(self.cameraSurface, self.drawOffset)

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

    def drawCones(self, drawLines=True, fillCone=True):
        """draw cones and their connections (closest cones drawn last)
            drawing connections is TBD (because i want to get the drawing order right)"""
        angleMargin = self.cameraFOV[0]/2 + self.FOVmargin
        nearbyConeList = self.mapToDraw.distanceToCone(self.mapToDraw.car.getRearAxlePos(), None, sortBySomething='SORTBY_DIST', 
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
                        dist, angle = GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), cone.connections[j].position)
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
                pointDistAngle = GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.target_list[i].position)
                pointDistAngle[1] = GF.radRoll(pointDistAngle[1]-self.mapToDraw.car.angle)
                if((abs(pointDistAngle[1]) < self.renderAngleMax) and (pointDistAngle[0] > self.renderDistMin)): #if current point lies ahead of (not behind) car
                    pointPixelPos = self.distAngleToPixelPos(pointDistAngle, 0.0)
                    #pygame.draw.circle(self.window, self.pathColor, pointPixelPos, int(self.pathCenterPixelDiam/2)) #draw center point (as filled circle, not ellipse)
                    pygame.draw.ellipse(self.window, self.pathColor, [GF.ASA(-(self.pathCenterPixelDiam/2), pointPixelPos), [self.pathCenterPixelDiam, self.pathCenterPixelDiam]]) #draw center point
            if(drawConeLines and (self.mapToDraw.target_list[i].coneConData is not None)): #instead of checking if pathFinderPresent, we can just check if the data is there (also more isRemote friendly)
                coneDistAngles = [GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.target_list[i].coneConData.cones[0].position), GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.target_list[i].coneConData.cones[1].position)]
                coneDistAngles[0][1] = GF.radRoll(coneDistAngles[0][1]-self.mapToDraw.car.angle);     coneDistAngles[1][1] = GF.radRoll(coneDistAngles[1][1]-self.mapToDraw.car.angle)
                if((abs(coneDistAngles[0][1]) < self.renderAngleMax) and (coneDistAngles[0][0] > self.renderDistMin) and (abs(coneDistAngles[1][1]) < self.renderAngleMax) and (coneDistAngles[1][0] > self.renderDistMin)): #if both cones lie ahead of (not behind) car
                    pygame.draw.line(self.window, self.pathColor, self.distAngleToPixelPos(coneDistAngles[0], 0.0), self.distAngleToPixelPos(coneDistAngles[1], 0.0), self.pathLineWidth) #line from left cone to right cone
            if(i > 0):#if more than one path point exists (and the forloop is past the first one)
                #draw line between center points of current pathline and previous pathline (to make a line that the car should (sort of) follow)
                if((abs(pointDistAngle[1]) < self.renderAngleMax) and (pointDistAngle[0] > self.renderDistMin)): #if current point lies ahead of (not behind) car
                    lastPointDistAngle = GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.target_list[i-1].position)
                    lastPointDistAngle[1] = GF.radRoll(lastPointDistAngle[1]-self.mapToDraw.car.angle)
                    if((abs(lastPointDistAngle[1]) < self.renderAngleMax) and (lastPointDistAngle[0] > self.renderDistMin)): #if last point lies ahead of (not behind) car
                        pygame.draw.line(self.window, self.pathColor, self.distAngleToPixelPos(lastPointDistAngle, 0.0), pointPixelPos, self.pathLineWidth) #line from center pos to center pos
        if(self.mapToDraw.targets_full_circle):
            endDistAngles = [GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.target_list[-1].position), GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.target_list[0].position)]
            endDistAngles[0][1] = GF.radRoll(endDistAngles[0][1]-self.mapToDraw.car.angle);     endDistAngles[1][1] = GF.radRoll(endDistAngles[1][1]-self.mapToDraw.car.angle)
            if((abs(endDistAngles[0][1]) < self.renderAngleMax) and (endDistAngles[0][0] > self.renderDistMin) and (abs(endDistAngles[1][1]) < self.renderAngleMax) and (endDistAngles[1][0] > self.renderDistMin)): #if both points lie ahead of (not behind) car
                pygame.draw.line(self.window, self.pathColor, self.distAngleToPixelPos(endDistAngles[0], 0.0), self.distAngleToPixelPos(endDistAngles[1], 0.0), self.pathLineWidth) #line that loops around to start
    
    def drawFinishLine(self, drawTopLine=True):
        """draw finish line (between finish_line_cones)"""
        if(len(self.mapToDraw.finish_line_cones) >= 2):
            coneDistAngles = [GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.finish_line_cones[0].position), GF.distAngleBetwPos(self.mapToDraw.car.getRearAxlePos(), self.mapToDraw.finish_line_cones[1].position)]
            coneDistAngles[0][1] = GF.radRoll(coneDistAngles[0][1]-self.mapToDraw.car.angle);     coneDistAngles[1][1] = GF.radRoll(coneDistAngles[1][1]-self.mapToDraw.car.angle)
            if((abs(coneDistAngles[0][1]) < self.renderAngleMax) and (coneDistAngles[0][0] > self.renderDistMin) and (abs(coneDistAngles[1][1]) < self.renderAngleMax) and (coneDistAngles[1][0] > self.renderDistMin)): #if both cones lie ahead of (not behind) car
                respectivePoses = [GF.distAnglePosToPos(coneDistAngles[0][0], coneDistAngles[0][1], np.zeros(2)), GF.distAnglePosToPos(coneDistAngles[1][0], coneDistAngles[1][1], np.zeros(2))]
                pygame.draw.line(self.window, self.finishLineColor, self.perspectiveProjection(respectivePoses[0], 0.0), self.perspectiveProjection(respectivePoses[1], 0.0), self.finishLineWidth)
                if(drawTopLine):
                    pygame.draw.line(self.window, self.finishLineColor, self.perspectiveProjection(respectivePoses[0], self.coneHeight), self.perspectiveProjection(respectivePoses[1], self.coneHeight), self.finishLineWidth)
    
    def drawStatText(self):
        """draw some usefull information/statistics on-screen"""
        newTime = time.time()
        if((newTime - self.statDisplayTimer)>self.statDisplayInterval):
            self.statDisplayTimer = newTime
            statsToShow = [] # a list of strings
            carToDraw = self.mapToDraw.car
            statsToShow.append(str(round(carToDraw.position[0], 2))+" , "+str(round(carToDraw.position[1], 2)))
            rearAxPos = carToDraw.getRearAxlePos()
            statsToShow.append(str(round(rearAxPos[0], 2))+" , "+str(round(rearAxPos[1], 2)))
            statsToShow.append(str(round(np.rad2deg(carToDraw.angle), 2)))
            statsToShow.append(str(round(carToDraw.velocity, 2))+"   "+str(round(np.rad2deg(carToDraw.steering), 2)))
            statsToShow.append(str(round(carToDraw.desired_velocity, 2))+"   "+str(round(np.rad2deg(carToDraw.desired_steering), 2)))
            if(self.mapToDraw.pathPlanningPresent and (carToDraw.pathFolData is not None)):
                statsToShow.append(str(carToDraw.pathFolData.auto)+"   "+str(carToDraw.pathFolData.laps))
            self.statRenderedFonts = [] # a list of rendered fonts (images)
            for textStr in statsToShow:
                self.statRenderedFonts.append(self.pygameFont.render(textStr, False, self.fontColor))
        for i in range(len(self.statRenderedFonts)):
            self.window.blit(self.statRenderedFonts[i], [self.drawOffset[0]+5,self.drawOffset[1]+5+(i*self.fontSize)])
    
    def redraw(self):
        """draw new frame"""
        self.cameraFrame()
        self.drawPathLines(True, self.mapToDraw.drawTargetConeLines)
        self.drawFinishLine()
        self.drawCones(True)
        self.drawFPScounter()
        self.drawStatText()





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
