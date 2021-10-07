#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../world/myHouse.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../world/myHouse.yaml initial_pose_a:=-1.57" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch"&
sleep 10
xterm -e " rosrun pick_objects pick_objects" 
