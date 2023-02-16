#!/bin/bash

# Create package directory
catkin_create_pkg put_odometry std_msgs rospy roscpp robot_localization xacro

# For playing rosbag
# cp /shared/fsds.bag /program/
# chmod +x fsds.bag

mv put_odometry src/put_odometry

# Create launch files
cd src/put_odometry
mkdir launch
cp /shared/config.launch launch/
cp /shared/config.yaml launch/

# Copy urdf files
mkdir visualization
cp /shared/config.xacro visualization/

# Add converter script
cp /shared/convert.py .
chmod +x convert.py

cd /program/

# Build package
catkin_make
source devel/setup.bash


# Run CAN
roscore &
roslaunch --wait put_odometry config.launch
