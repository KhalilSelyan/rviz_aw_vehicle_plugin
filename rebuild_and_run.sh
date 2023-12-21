#!/bin/bash

# Kill existing rviz2 process
pkill -f rviz2

# Build the packages
colcon build --packages-select rviz_2d_overlay_msgs rviz_2d_overlay_plugins

# Source the setup script
source ./install/setup.bash

# Launch rviz2 with stdout and stderr printed to the terminal
rviz2
