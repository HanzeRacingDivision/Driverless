FROM ros:melodic

# Update outdated registry key
RUN sudo apt-get -y update && sudo apt-get -y upgrade

# Install ROS essentials
RUN sudo apt-get -y install libeigen3-dev libcppunit-dev python3-psutil python3-future \
                            python3-pip python-pip curl wget python3-tk

# Install robot-localization dependencies
RUN sudo apt-get -y install ros-melodic-robot-localization ros-melodic-rviz ros-melodic-cv-bridge \
                            ros-melodic-map-server ros-melodic-xacro

# Create directory for program and files shared with host
RUN mkdir -p /program/src
RUN mkdir -p /shared

# Install python requirements
RUN pip3 install --upgrade pip
COPY requirements.txt /program/requirements.txt
RUN pip3 install -r /program/requirements.txt

# Copy startup and refresh scripts
COPY entrypoint.sh /program/

# Change permissions
RUN chmod 777 /program/entrypoint.sh

WORKDIR /program
