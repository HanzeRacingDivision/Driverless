# Solved Issues
## Clogged up topic for odometry
### Description
The topic /fsds/testing_only/odom was getting clogged up, when being read from the simulation. The data was delayed, which influenced the processing.
### Solution 1
Changing the frequency of publishin go the topic in the launch file:
Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/launch
In this line the value:
'''
<param name="update_odom_every_n_sec" type="double" value="1" />
'''
### Solution 2
Lowering the size of the queue of the simulator
In the file ~/Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp,
change in the next lines the argument after "testing_only/odom" to the desired size of the queue.
'''
if(!competition_mode_) {
    odom_pub = nh_.advertise<nav_msgs::Odometry>("testing_only/odom", 1);
    track_pub = nh_.advertise<fs_msgs::Track>("testing_only/track", 10, true);
		extra_info_pub = nh_.advertise<fs_msgs::ExtraInfo>("testing_only/extra_info", 10, true);
}
'''
