#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select project_description project_moveit_config project_bringup octomap_rviz_plugins
source install/setup.bash
source /opt/ros/${ROS_DISTRO}/setup.bash
exec "$@"
