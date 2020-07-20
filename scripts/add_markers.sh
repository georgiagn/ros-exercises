#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roslaunch turtlebot_gazebo  turtlebot_world.launch " &
sleep 10
xterm  -e  "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch " & 
sleep 10
xterm -e "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e "source /opt/ros/kinetic/setup.bash; source devel/setup.bash; rosrun add_markers add_markers_fixedtime " &

