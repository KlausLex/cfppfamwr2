#!/bin/bash

ENV="
-e DISPLAY \
"

HW="
--privileged \
--net=host \
--ipc=host \
--device=/dev/dri:/dev/dri \
"
VOL="
-v /tmp/.X11-unix:/tmp/.X11-unix:rw
-v ${PWD}/src/project_description/:/opt/ros2_ws/src/project_description
-v ${PWD}/src/project_moveit_config/:/opt/ros2_ws/src/project_moveit_config
-v ${PWD}/src/project_bringup/:/opt/ros2_ws/src/project_bringup
-v ${PWD}/src/pc_point_fetcher/:/opt/ros2_ws/src/pc_point_fetcher
-v ${PWD}/src/target_planner/:/opt/ros2_ws/src/target_planner
-v ${PWD}/src/collision_environment/:/opt/ros2_ws/src/collision_environment
"

NAME="final_project:latest"

xhost +
docker run -it --rm $ENV $HW $VOL $NAME \
    bash
