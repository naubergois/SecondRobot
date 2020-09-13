#!/bin/bash

source /home/workspace/catkin_ws/devel/setup.bash

# Build the catkin_ws
cd $(pwd)/../..; catkin_make

xterm  -e "source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/nauber5.world" &
sleep 15

xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/my_robot/maps/nauber5.yaml" &
sleep 15

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 15


xterm -e "rosrun add_markers add_markers" &
sleep 15
