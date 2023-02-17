### Starting the simulator
```bash
cd $HOME
cd Formula-Student-Driverless-Simulator
bash FSDS.sh
```
Now choose map and click start simulator using the buttons in the window that pops up.

### Start the ROS bridge
```bash
cd $HOME
cd Formula-Student-Driverless-Simulator
cd ros
source devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch
```
Now the bridge should be running, you should be getting messages about how each topic is doing.

If you want to be fancy and have plotso of all variable then use
```bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch plot:=true
```

### Listening to ROS topics
```bash
source devel/setup.bash
```
#### lidar pointcloud
```bash
rostopic echo /fsds/lidar/Lidar
```
#### GPS x, y, z
```bash
rostopic echo /fsds/gps
```
#### IMU
```bash
rostopic echo /fsds/imu
```
#### All cone positions
```bash
rostopic echo /fsds/testing_only/track
```

### Creating subsribers
```bash
cd Formula-Student-Driverless-Simulator
cd ros
cd src
```
#### Make environment
```bash
catkin_create_pkg testingsim std_msgs rospy roscpp fs_msgs
cd ..
catkin build # rebuild the environment
source devel/setup.bash # source again
```
#### Now add your scripts to testingsim
```bash
cd src/testinsim/src # in this dir add the files
```
#### Make them executable
```bash
chmod +x *
```
#### Run them
```
rosrun testingsim name_file.py
```
