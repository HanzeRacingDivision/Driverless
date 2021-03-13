python requirements: <br/>
- pygame 2.0.0 (older versions dont have SDL2, which determines how cursor-sprite changes are handled) <br/>
- numpy (any version) <br/>

how to run: <br/>
  run coneConnecting.py

how to use: <br/>
  (you can resize the window by just dragging it like any other windows window) <br/>
cones:
- left mouse button places left cones (yellow)
- right mouse button places right cones (blue)
- holding shift while placing cones makes them attempt to connect to a nearby (suitable) cone
- clicking on placed cones (with either mouse button) makes it try to connect to a nearby suitable cone
- holding F while placing/clicking on a cone will make that cone a finish-line-cone (there can only be 1 left and 1 right finish cone) <br/>
camera:
- press C to switch between regular and car-centered camera
- scrolling zooms in and out
- CTRL+scrolling rotates the screen (only in car-centered camera)
- middle mouse buttom moves the viewpoint (dragging like google maps) (not in car-centered camera) <br/>
other:
- pressing R makes it try to find a path (requires connected cones on both side of track
- pressing L makes it rewrite the coneLog immidielty (also happens automatically, but this just forces it to happen) <br/>
driving (temporary):
- arrow keys

meaning of the colors and other visual aspects: <br/>
- yellow cones are yellow circles, connections between them are yellow lines. Same for blue cones except in blue...
- holding F (finish button) makes your cursor into a racing-finish flag (fun, right)
- if there are 2 valid finish cones, a red (finish) line will be drawn between them
- (currently) the car is a green box with a red arrow, indicating the forward direction


how to run with a previously saved track (coneLog CSV file): <br/>
- open the coneConnecting.py window and drag&drop the file in there  OR
- use the command line and enter the coneLog filepath as an argument  OR
- make a windows shortcut, put the location of your python installation at the start of the Target section of the properties of the windows shortcut and just drag&drop the file onto the shortcut <br/>

running multiple ones in paralel: (not threaded (yet)) <br/>
- use coneConnecting multi.py <br/>
- it imports coneConnecting.py as a library and makes N by N instances of it <br/>
- you can also use the multi sketch to import several coneLogs at once. It will automatically make enough instances <br/>
