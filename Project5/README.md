# Home Service Robot
The Home Service Robot is the final project of the Udacity Robotics Software Engineer nanodegree. 

## Official ROS packages
This project utilizes the following ROS packages, cloned all of them on October 9, 2021.


- [gmapping](http://wiki.ros.org/gmapping): With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop): With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers): With the view_navigation.launch file, you can load a preconfigured rviz workspace.
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo): With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

This ROS Packages allow perform the following functionalities:
* Localization: ACML algorithm was used provided in turtlebot_simulator package lunching amcl_demo.launch, this package performs Advanced Monte Carlo Localization, an algorithm that uses particle filters to locate our robot. 
* Mapping: The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.
In this opportunity, the map created in the before projects was used, so occupancy grid map was created in project “Where Am I?” using ROS package: pgm_map_creator.

* Navigation: This packages implements the Navigation Stack which allows us to send a navigation goal for our robot, the underlying algorithm used for path planning is Dijkstra's Algorithm.


## Test nodes developed
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

Now that the world is mapped and the robot can follow commands, the robot can be instructed to pick up and drop off a simulated object at different waypoints. 
First two packages were created: add_markers and pick_objects. The latter will send a message to the robot so it knows its two destinations, both pick-up point and drop-off point. The former will just publish the marker so it can be seen in Rviz before the pick-up and after the drop-off. The add_markers node will be responsible for checking if the virtual object has been picked up by subscribing itself to the /odom topic and validating how near is to the goals. At the same time the environment around the robot will be mapped, the robot will be localized and it will know how to reach each goal by the previously mentioned stacks combined.
To test this, run the `home_service.sh` script:


```shell
$ ./home_service.sh
```

![alt-text](https://github.com/abenso/Udacity/blob/master/Project5/images/home_service.gif)
