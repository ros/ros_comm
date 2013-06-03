#!/bin/bash

DROP_PROBABILITY=0.01
IP=192.168.99.99

RULE="-i lo -d $IP -m statistic --mode random --probability $DROP_PROBABILITY -j DROP"

echo "Creating an additional loopback IP for testing..."
sudo ifconfig lo:1 $IP

echo "Using iptables to drop packets at a set probability..."
sudo iptables -A INPUT $RULE

echo "Running the test..."
export ROS_HOSTNAME=$IP
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
roslaunch test_roscpp fragmented_udp_data.launch

echo "Cleaning up..."
sudo iptables -D INPUT $RULE
sudo ifconfig lo:1 down
