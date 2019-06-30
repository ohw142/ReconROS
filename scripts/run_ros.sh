#!/bin/bash
export ROS_MASTER_URI=http://192.168.2.99:12543/
export ROS_IP=192.168.2.99
source /opt/ros/kinetic/setup.bash
source /home/xilinx/Development/ros/devel/setup.bash
roscore -p 12543 &
sleep 15
rosrun reconros_car_serial serial_node.py &
sleep 15
rosrun reconros_car_example_controller controller_node.py & 
sleep 15
while kill -0 $(pgrep roscore) 2> /dev/null; do sleep 1; done;
