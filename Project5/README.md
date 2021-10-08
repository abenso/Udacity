# Home Service Robot
The Home Service Robot is the final project of the Udacity Robotics Software Engineer nanodegree. 

## Official ROS packages
This project utilizes the following ROS packages, cloned all of them on October 9, 2021.

- [gmapping](http://wiki.ros.org/gmapping): With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop): With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers): With the view_navigation.launch file, you can load a preconfigured rviz workspace.
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo): With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

## Package Tree
```
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── map                          # world files
    │   ├── ...
    ├── Scripts                   # shell scripts files
    │   ├── ...
    ├──RvizConfig                      # rviz configuration files
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```

## Install Packages
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ sudo apt-get update
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping
$ git clone https://github.com/turtlebot/turtlebot
$ git clone https://github.com/turtlebot/turtlebot_interactions
$ git clone https://github.com/turtlebot/turtlebot_simulator
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosdep -i install gmapping
#All required rosdeps installed successfully
$ rosdep -i install turtlebot_teleop
#All required rosdeps installed successfully
$ rosdep -i install turtlebot_rviz_launchers
#All required rosdeps installed successfully
$ rosdep -i install turtlebot_gazebo
#All required rosdeps installed successfully
$ catkin_make
$ source devel/setup.bash
```



### Part 1: SLAM

The first thing the robot can do is simultaneous localization and mapping (SLAM). To perform SLAM, run the `test_slam.sh` script: This is an example because I used the map created in Project 3 

```shell
$ ./test_slam.sh
```

### Part 2: Navigation

The next task for the robot is navigation. To test the robot's navigation capabilities, run the `test_navigation.sh` script:

```shell
$ ./test_navigation.sh
```

In this test you'll see the robot in a completed map in Rviz. Click the "2D Nav Goal" button and click/drag somewhere on the map to command the robot. The robot will find a path to the goal location and follow it.

### Part 3: Full Service

Now that the world is mapped and the robot can follow commands, the robot can be instructed to pick up and drop off a simulated object at different waypoints. To do this, run the `home_service.sh` script:

```shell
$ ./home_service.sh
```

An item (represented by a green cube) will show up in Rviz. The robot will navigate to the item, at which point it will disappear (indicating it has been picked up), and then the robot will navigate to another point and drop off the item, at which point the item will reappear.

![alt-text](https://github.com/abenso/Udacity/blob/master/Project5/images/home_service.gif)
