# PUT_DV Odometry

This repository has three branches:

* [main](https://github.com/PUT-Motorsport/PUTM_DV_Odometry) - Docker image with `robot_localization` and fusion based on `CAN` communication
* [plotting](https://github.com/PUT-Motorsport/PUTM_DV_Odometry/tree/plotting) - Docker image doing the same as above, but instead of using real data pulling information from FS Student simulation
* [gps_imu](https://github.com/PUT-Motorsport/PUTM_DV_Odometry/tree/gps_imu) - local package for GPS (Groove or Ardusimple RTK) fusion with IMU using `robot_localization`

## Main package

ROS package for calculating position of the robot based on GPS odometry and IMU.
Packages used:
* [robot_localization](http://wiki.ros.org/robot_localization)
* [ros2can](https://github.com/PUT-Motorsport/PUTM_DV_utils_2020/tree/can)

## Installation

### With Docker (recommended)

The project is by default made with an intention to run inside [Docker](https://www.docker.com/) on **Linux** container for testing.

a) Clone the repository

b) Set **VOLUME** and **HOME** variable to the location of `shared` folder and home directory:
```bash
$   export VOLUME="location/here"
$   export HOME="/home/username"
```

c) Start the project by running the `start.sh` script:
```bash
$   ./start.sh
```

Docker is multi-platform software, so you can convert the `.sh` script to the script format your system supports.

d) Once the container is up, start the project:
```bash
$   ./entrypoint.sh
```

### Without Docker

If you really insist on running this project without Docker, download all requirements for virtual CAN:

```bash
$   sudo apt-get -y install libeigen3-dev libcppunit-dev python3-psutil python3-future \
                            python3-pip python-pip curl wget
```

Then all requirements for robot_localization package:

```bash
$   sudo apt-get -y install ros-melodic-robot-localization ros-melodic-rviz ros-melodic-cv-bridge
```

Install all python packages required to run ros2can:
```bash
$   pip3 install -r /program/requirements.txt
```

Create `/program` folder or change 15th line in `entrypoint.sh` file to the location of your project.

Run the script:
```bash
$   ./entrypoint.sh
```
