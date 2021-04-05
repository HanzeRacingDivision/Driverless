python requirements:
- pygame 2.0.1 (if a newer version doesnt work let Thijs know)
- (only for remote visualisation) socket (python builtin)
- (only for real-life model car control) pyserial (only in carMCUclass.py)

how to use: <br/>
- see 'generalSim.py' for an example

cones:
- left mouse button places left cones (yellow)
- right mouse button places right cones (blue)
- holding F while placing/clicking on a cone will make that cone a finish-line-cone (there can only be 1 left and 1 right finish cone)
- holding shift while placing cones makes them attempt to connect to a nearby (suitable) cone (ONLY IF coneConnecting.py is present)
- clicking on placed cones (with either mouse button) makes it try to connect to a nearby suitable cone (ONLY IF coneConnecting.py is present)

camera:
- press V to switch between regular and car-centered camera
- scrolling zooms in and out
- CTRL+scrolling rotates the screen (only in car-centered camera)
- middle mouse buttom moves the viewpoint (dragging like google maps) (not in car-centered camera)

other:
- pressing P makes it try to find a path (requires connected cones on both side of track) (ONLY IF pathFinding.py is present)
- pressing C clears the car position history line, C+CTRL clears everything
- pressing A toggles auto-driving (only has effect if pathPlanning(Temp).py is present)
- pressing +/- increases/decreases auto-driving target speed
- pressing Q toggles drawing qubic splines (only has effect if pathPlanning(Temp).py is present)
- pressing T toggles whether the cone-lines that determined the targets are drawn (only has effect if pathFinding.py is present)

driving (temporary): COMMENTED OUT AT BOTTOM OF drawDriverless.py
- arrow keys (only if autodriving is off, and has no effect on real car (if realCar is connected))


meaning of the colors and other visual aspects: <br/>
- yellow cones are yellow circles, connections between them are yellow lines. Same for blue cones except in blue...
- holding F (finish button) makes your cursor into a racing-finish flag (fun, right)
- if there are 2 valid finish cones, a red (finish) line will be drawn between them
- (currently) the car is a green box with a red arrow, indicating the forward direction
