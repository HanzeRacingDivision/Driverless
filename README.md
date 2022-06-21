
NOTE: this branch is depricated, as we'll be moving to Alex's sim (because things like the (new EKF) SLAM and RL code are all written for that).

# Status post-deprication
What I'm currently working is to make Alex's sim capable of running on a real car (in real-time). I'm gonna be honest, i don't love it, becuase i spent a lot of time writing my version, but also (subjectively) mine is written for real-car operation first-and-foremost and as a simulation second. It's sofar proven to be a bitch-and'a-half converting Alex simulation while changing as little as possible (to make sure all the SLAM/RL code has a chance of being compatible).

# Status POST-post-deprication
i got bored and made another version. No more multicore, no more remote map viewing (for now?). Since the LiDAR will not be handled by the PC anymore (i hope), i changed the LiDAR code to reflect this. You can still simulate lidar blobs for now. Because python cannot really multi-thread, this version works a lot faster than the last one.

# Overview
Python code for a selfdriving car. Can be used for both real-life and simulation purposes <br/>
I'm reusing the old (already depricated) branch to once again host my stubborn simulation desires.
# Status
This version does everything it used to (coneconnecting, pathfinding, pathplanning (simple auto-driving and making splines), lidar handling&sim, DIY_SLAM and 3Dish rendering (over a camera frame(?))), but now once again single-core. It's fast, it's widely considered to be illegible to those unwilling to dive deep, and it runs DAMN GOOD on real-life hardware.

# How to run
use the "ARCv_.py" file to launch the whole thing (the rest are only components of it)

requirements:
 - python 3.8+? untested on anything below 3.8
 - numpy 
 - pandas (for map_loader)
 - pygame 2.0.1 (for map visualization)
 - pyserial (only for real-life, not for simulations) (note: used as "import serial")
 - scipy (for cubic splines (pathPlanning))
 - PIL (a.k.a. Python Image Library, a.k.a. Pillow, only used for rendering headlights)
 - (OPTIONAL!) numba (used to make some code faster, there are always NoNumba versions of things though)
numba is recommended, but there are backup files in case you're having difficulty installing it (understandable on a Rpi/Jetson)
if map-loading is not working as it should, you may need to 'pip install openpyxl'. try loading the file as a cmdline argument (instead of drag-dropping) to get better feedback.

# How to use
controls:
the mouse can be used to:
 - move the viewpoint (hold MMB (scrollwheel) and drag mouse)
 - zoom (scroll)
 - place cones (LMB for left cones, RMB for right cones)
 - remove cones (hold 'R') (hold SHIFT to erase simVars cone as well)
 - connect cones (click on an existing cone, or place while holding SHIFT)
 - set the finish cones (hold 'F' and click on (or place) a cone)
keyboard functions:
 - 'P' to create a path (automatically make a (limited) path
 - 'A' toggle auto-driving (start/stop driving)
 - '-' reduce target speed
 - '+' increase target speed ('=' also works)
 - 'Q' toggle cubic spline (currently does nothing, just for visuals)
 - 'T' show relation between cones and target points (path)
 - 'L' switch lidar visual debug mode (spotcount, spotcount+points, coneID)
 - 'H' toggle headlights (does nothing, just for visuals) and (temporarily) also toggles the car sprite
 - 'D' custom debug stuff, don't worry about that right now
 - 'C' toggle car position history-line (histogram?), note: recudes FPS
 - 'V' switch view mode (global cam with dragging or car-following cam ('CTRL'+scrolling rotates view))
 - 'CTRL'+'V' toggle 3D view mode (camera overlay) note: currently missing camera footage
 - 'S' save current map to file
 - '[ and ]' (only for remote interface) increase/decrease packet rate
 - 'M' (only for simulations) toggle undiscoveredCones (whether loaded/placed cones are added to the conelists or yet to be discovered)
 - 'Z' toggle how zooming (mouse scrolling) works. the defualt mode is nice for trackpads (as you can't drag)
other:
 - you can drag & drop files onto the pygame window to load mapfiles
 - you can load mapfiles as an argument (in the commandline when starting python)

the visualization code was built to (among other things) handle having multiple driverless instances running in paralel(ish). You could, with minimal effort, create an interface for running multiple driverless instances at the same time (for stuff like training AI's in real-time, visually)

# References
Most of this shit just comes from my brain (becuase i was too lazy to find real references). The SLAM_DIY is based on my rejection of EKF SLAM (the old one, before Yasmin fixed it), and my desire to consider multiple cones at once when calculating the positional error of the car (because how else are you going to differentiate positional and rotation error...)
I integrated a few of Alex's ideas/code: cubic splines (althought they don't do anything in mine yet), the car PNG (from google) and using pandas for fancy .xlsx map files.
