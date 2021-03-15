import pygamesim as pgs


enableLogging = True
numberOfSimsRoot = 2 #this value squared is the actual number of sims, this is just how many rows and columns of sims there are
#file importing
fileToImport = [['' for j in range(numberOfSimsRoot)] for i in range(numberOfSimsRoot)]
if(len(pgs.sys.argv) == 2):
    if(type(pgs.sys.argv[1]) is str):
        if(pgs.sys.argv[1].endswith('.csv')):
            print("found sys.argv[1] with a '.csv' extesion, attempting to import:", pgs.sys.argv[1])
            fileToImport = [[pgs.sys.argv[1] for j in range(numberOfSimsRoot)] for i in range(numberOfSimsRoot)]
        else:
            print("found sys.argv[1] but does not have '.csv' extension, so NOT importing that shit")
elif(len(pgs.sys.argv) > 2):
    print("multiple sys.argv's found, so attempting to import all:", len(pgs.sys.argv)-1)
    filesToImport = []
    for sysArg in pgs.sys.argv[1:]:
        if(type(sysArg) is str):
            print("attempting to import", sysArg.split('\\')[-1]) #remove the stuff before the filename for easier debugging
            if(sysArg.endswith('.csv')):
                filesToImport.append(sysArg)
            else:
                print("sysArg but does not have '.csv' extension, so NOT importing that shit")
        else:
            print("sysarg not string")
    numberOfSimsRoot = int(len(filesToImport)**0.5) #round down (cut off everthing after the decimal point, 1.6 -> 1 amd 1.2 -> 1)
    numberOfSimsRoot += (1 if (int(numberOfSimsRoot**2) < len(filesToImport)) else 0) #this ensures the number of sims is always equal to or greater than the number of input files
    fileToImport = [[(filesToImport[i*numberOfSimsRoot+j] if ((i*numberOfSimsRoot+j)<len(filesToImport)) else '') for j in range(numberOfSimsRoot)] for i in range(numberOfSimsRoot)]

pgs.pygameInit()

multiSims = []
displaySize = [pgs.pygame.display.Info().current_w, pgs.pygame.display.Info().current_h]

bgColorList = [[50,50,50], [20,50,50], [50,50,20], [20,50,20], [50,30,50], [20,30,50], [50,30,20], [20,30,20], [80,80,80]]

for i in range(numberOfSimsRoot):
    multiSims.append([])
    for j in range(numberOfSimsRoot):
        multiSims[i].append(pgs.pygamesim(pgs.window, [], \
                                          (displaySize[0]/numberOfSimsRoot, displaySize[1]/numberOfSimsRoot), \
                                          ((displaySize[0]/numberOfSimsRoot)*i, (displaySize[1]/numberOfSimsRoot)*j), \
                                          [0,0], 0, 10, False, True, fileToImport[i][j], enableLogging, "pygamesim["+str(i)+"]["+str(j)+"]"))
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