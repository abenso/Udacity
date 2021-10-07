#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../map/myHouse.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../map/myHouse.yaml initial_pose_a:=-1.57" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" 
sleep 5
xterm -e " rosrun add_markers add_markers" 
