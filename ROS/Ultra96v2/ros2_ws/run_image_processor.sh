#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/mp4d/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=69
ros2 run image_processor image_processor_node
