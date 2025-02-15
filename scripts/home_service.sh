#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/ros-exercises/src/my_robot/worlds/my.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/ros-exercises/src/my_robot/maps/map.yaml

xterm -e "roslaunch turtlebot_gazebo  turtlebot_world.launch " &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch " & 
sleep 10
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e "rosrun add_markers add_markers " &
xterm -e "rosrun pick_objects pick_objects " &

