#!/usr/bin/env bash

# This should be the ip address for this computer
host_ip="192.168.0.52"

# This should be the ip address for the computer who is controlling the robot
boris_host_ip="192.168.0.52"



# Exporting ROS variables  appropriately (see http://wiki.ros.org/ROS/NetworkSetup)

# not using hostname (either use hostname or ROS_IP, here we choose ROS_IP)
unset ROS_HOSTNAME

export ROS_IP="${host_ip}"

export ROS_MASTER_URI="http://${boris_host_ip}:11311"



