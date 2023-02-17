# Setting up the simulator

## get OS Ubuntu 20.04

## intall all this stuff

```bash
# use sudo apt-get install
python #(make sure it is python3 and it is callable as python)
git
git lfs
ros-noetic-desktop-full # for problems see http://wiki.ros.org/noetic/Installation/Ubuntu
ros-noetic-tf2-geometry-msgs
python-catkin-tools
ros-noetic-rqt-multiplot
ros-noetic-joy
ros-noetic-cv-bridge
ros-noetic-image-transport
libyaml-cpp-dev
libcurl4-openssl-dev
```

## clone the repository with the simulator
```bash
git lfs clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git
```

## checkout the branch corresponding to the release
```bash
git checkout tags/v2.1.0
```

## get the release of the executable from GitHub
[GitHub](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.1.0)
download the linux release, extract the contents into the cloned repository of you created in the previous step.


## setup ros
```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## setup ros for FS-Driverless
```bash
source /opt/ros/noetic/setup.bash
cd Formula-Student-Driverless-Simulator/ros
catkin init
catkin config --cmake-args
catkin build

```
