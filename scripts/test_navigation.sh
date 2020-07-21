#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/ros-exercises/src/my_robot/worlds/my.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/ros-exercises/src/my_robot/maps/map.yaml

xterm  -e  "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roslaunch turtlebot_gazebo  turtlebot_world.launch " &
sleep 10
xterm  -e  "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch " & 
sleep 5
xterm -e "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &

