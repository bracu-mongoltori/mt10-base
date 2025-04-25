#!/bin/bash

# This script is used to control a rover using a joystick
cmd1="ros2 run joy joy_node"
cmd2="ros2 run rover_node rover_run"
cmd3="ros2 run control_node Controller"

# Source ROS2 environment (adjust path to your ROS2 installation)
source /opt/ros/humble/setup.bash

# Function to launch commands in new tabs
launch_in_tab() {
    gnome-terminal --tab -- bash -c "echo 'Running: $1'; \
    source ~/.bashrc; \
    $1; \
    echo 'Press Enter to close...'; \
    read"
}

# Launch all components
launch_in_tab "$cmd1"
launch_in_tab "$cmd2"
launch_in_tab "$cmd3"