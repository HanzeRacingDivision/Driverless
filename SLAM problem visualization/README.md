'SLAM problem visualization' is a little (pygame) sketch to help me debug my own DIY SLAM. It considers the error on the car to be the a linear (x,y) offset, plus a rotational error.

I attempt to solve for this error, by first calculating the linear offset using 'multilateration' (using distance measurements to find a position with minimal error (least squares), like GPS), and then calculating rotational offset (which is very simple if the measured cones are the same distance from the car as the known cones).

The cones (and their error) are generated randomly. You can enable/disable linearTimeOffsets (whether older cone measurements are more accurate, due to increasing error over time) and whether or not sensor-error exists (see line 240ish).
The whole process is visualized in several scenes, each showing 1 step of the process.

As for the real car; this math will turn into my own DIY_SLAM, which i'm making because i got fed up with EKF_SLAM, and it's inability to consider multiple landmarks at once in particular.
I hope to one day merge some of this math into a better (existing/EKF) SLAM implementation, as this approach has no real filtering.
The most important thing that SLAM needs to accomplish in the current (aug/sept 2021) car, is to keep orientational error to a minimum. The car's rotational actuators/sensors/math is pretty shit, so any help from SLAM could be a huge improvement already.

controls:
* 'R' to randomly generate a new scenario
* '9' and '0' to switch to the last/next scene
* 'T' to toggle linearTimeOffsets (debug), please also reload using 'R' after
* use the mouse wheel zoom (scroll) and move around (by pressing it)

requirements / how-to-run:
* i ran it with:
  * pygame 2.0.1 (may matter a little, but mostly for cursor-sprite reasons)
  * python 3.8  (shouldn't matter)
  * numpy and math libraries (any version should be fine)
* download
  * SLAM_problen_visualization.py
  * multilateration.py
  * the GF folder
* run SLAM_problen_visualization.py
