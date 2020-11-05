import pygamesim as pgs


pgs.pygameInit()

multiSims = []
numberOfSimsRoot = 3 #this value squared is the actual number of sims, this is just how many rows and columns of sims there are
displaySize = [pgs.pygame.display.Info().current_w, pgs.pygame.display.Info().current_h]

bgColorList = [[50,50,50], [20,50,50], [50,50,20], [20,50,20], [50,30,50], [20,30,50], [50,30,20], [20,30,20], [80,80,80]]

enableLogging = False

fileToImport = ''
if(len(pgs.sys.argv) > 1):
        if(type(pgs.sys.argv[1]) is str):
            if(pgs.sys.argv[1].endswith('.csv')):
                print("found sys.argv[1] with a '.csv' extesion, attempting to import:", pgs.sys.argv[1])
                fileToImport = pgs.sys.argv[1]

for i in range(numberOfSimsRoot):
    multiSims.append([])
    for j in range(numberOfSimsRoot):
        multiSims[i].append(pgs.pygamesim(pgs.window, [], \
                                          displaySize[0]/numberOfSimsRoot, displaySize[1]/numberOfSimsRoot, \
                                          (displaySize[0]/numberOfSimsRoot)*i, (displaySize[1]/numberOfSimsRoot)*j, \
                                          10, True, fileToImport, enableLogging, "pygamesim["+str(i)+"]["+str(j)+"]"))
        multiSims[i][j].bgColor = bgColorList[(i*numberOfSimsRoot)+j]
        multiSims[i][j].addCar()

while pgs.windowKeepRunning:
    pgs.handleAllWindowEvents(multiSims) #handle all window events like key/mouse presses, quitting and most other things
    for i in range(numberOfSimsRoot):
        for j in range(numberOfSimsRoot):
            multiSims[i][j].redraw()
    pgs.frameRefresh() #send (finished) frame to display

print("closing logging file(s)...")
if(enableLogging):
    for i in range(numberOfSimsRoot):
            for j in range(numberOfSimsRoot):
                multiSims[i][j].closeLog()
pgs.pygameEnd()