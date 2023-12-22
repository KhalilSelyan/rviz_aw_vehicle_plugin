#!/bin/bash

# Source the ROS 2 setup script
source /opt/ros/humble/setup.bash

# Source autoware setup script (to be edited depending on your installation)
source /home/khalil/autoware/install/setup.bash

# Build the packages
colcon build --packages-select rviz_2d_overlay_msgs rviz_2d_overlay_plugins

# Source the setup script
source ./install/setup.bash

# Launch rviz2
# rviz2 -l --ros-args --log-level DEBUG
rviz2
