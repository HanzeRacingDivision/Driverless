
NOTE: this branch is basically depricated, as we'll be moving to Alex's sim (because things like the (new EKF) SLAM and RL code are all written for that).

# Status post-deprication
What I'm currently working is to make Alex's sim capable of running on a real car (in real-time). I'm gonna be honest, i don't love it, becuase (subjectively) i spent a few dozen hours writing the thousends of lines of code in my version, but also because (objectively) mine is written for real-car operation firstly and as a simulation second. It's sofar proven to be a bitch-and'a-half converting Alex simulation while changing as little as possible (to make sure all the SLAM/RL code has a chance of being compatible).

# Overview
Python code for a selfdriving car. Can be used for both real-life and simulation purposes <br/>
I (thijs) made this branch becuase of the major overhaul i did (in july 2021). The code in the 'thijs-update' is largely similar, but far more single-core (and a few more bugs, hopefully). I've used this code to run the scale RC car in my minor presentation (September 2021) and at the Hanze open-day (November 2021).
# Status
This version does everything it used to (coneconnecting, pathfinding, pathplanning (simple auto-driving and making splines)) and more (lidar, 3Dish rendering (over a camera frame(?), bugfixes), while making as much use of the python multiprocessing library as possible (for true multicore (not just multithread singlecore) performance). The downside of this is that python is really not meant for any of it. Switching to another (faster) language is mostly hindered by the need to find an alternative to pygame. The speed of the code is now hampered by the fact that it needs to spend a lot of time on inter-core communications (which all involve pickling, because python won't let you share objects directly).

# How to run
use the "july.py" file to launch the whole thing (the rest are only components of it)

requirements:
 - python 3.8+ (must be 3.8+ for shared_memory (multiprocessing))
 - numpy 
 - pandas (for map_loader)
 - pygame 2.0.1 (for map visualization)
 - pyserial (only for real-life, not for simulations) (note: used as "import serial")
 - scipy (for cubic splines (pathPlanning))
 - PIL (a.k.a. Python Image Library, a.k.a. Pillow, only used for rendering headlights)
 - numba (used to make some code faster, there are usually NoNumba versions of things though)
numba is recommended, but there are backup files in case you're having difficulty installing it (understandable on a Rpi/Jetson)
if map-loading is not working as it should, you may need to 'pip install openpyxl'. try loading the file as a cmdline argument (instead of drag-dropping) to get better feedback.
note: on windows, multithreaded python only prints the main thread's data, while on linux, all threads's print() data is shown.

# How to use
controls:
the mouse can be used to:
 - move the viewpoint (hold MMB (scrollwheel) and drag mouse)
 - zoom (scroll)
 - place cones (LMB for left cones, RMB for right cones)
 - remove cones (hold 'R')
 - connect cones (click on an existing cone, or place while holding 'shift')
 - set the finish cones (hold 'F' and click on (or place) a cone)
keyboard functions:
 - 'P' to create a path (automatically make a (limited) path
 - 'A' toggle auto-driving (start/stop driving)
 - '-' reduce target speed
 - '+' increase target speed ('=' also works)
 - 'Q' toggle cubic spline (currently does nothing, just for visuals)
 - 'T' show relation between cones and target points (path)
 - 'L' switch lidar visual debug mode (spotcount, spotcount+points, coneID)
 - 'H' toggle headlights (does nothing, just for visuals) (also only works if car sprite works)
 - 'D' custom debug stuff, don't worry about that right now
 - 'C' toggle car position history-line (histogram?), note: recudes FPS
 - 'V' switch view mode (global cam with dragging or car-following cam ('CTRL'+scrolling rotates view))
 - 'CTRL'+'V' toggle 3D view mode (camera overlay) note: currently missing camera footage
 - 'S' save current map to file
 - '[ and ]' (only for remote interface) increase/decrease packet rate
other:
 - you can drag & drop files onto the pygame window to load mapfiles
 - you can load mapfiles as an argument (in the commandline when starting python)

the visualization code was built to (among other things) handle having multiple driverless instances running in paralel(ish). You could, with minimal effort, create an interface for running multiple driverless instances at the same time (for stuff like training AI's in real-time, visually)

# References
Most of this shit just comes from my brain (becuase i was too lazy to find real references). The SLAM_DIY is based on my rejection of EKF SLAM (the old one, before Yasmin fixed it), and my desire to consider multiple cones at once when calculating the positional error of the car (because how else are you going to differentiate positional and rotation error...)
I integrated a few of Alex's ideas/code: cubic splines (althought they don't do anything in mine yet), the car PNG (from google) and using pandas for fancy .xlsx map files.
