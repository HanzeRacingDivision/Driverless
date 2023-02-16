#!/bin/bash

modprobe can_dev
modprobe can_raw
modprobe vcan
ip link add dev vcan0 type vcan
ip link set up vcan0
sudo docker run -it -v $VOLUME:/shared \
                --env="DISPLAY=${DISPLAY}" \
                --env="NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}" \
                --env="NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics" \
                --runtime=nvidia \
                --net=host \
                --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
                --privileged \
                ros:latest
