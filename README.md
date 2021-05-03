# First assignment _Research Track_ course - second module 
## Table of Contents
- [First assignment _Research Track_ course - second module](#first-assignment-research-track-course---second-module)
  - [Table of Contents](#table-of-contents)
  - [General Instructions](#general-instructions)
  - [More about the "_action_" branch](#more-about-the-action-branch)
  - [More about the "_ros2_" branch](#more-about-the-ros2-branch)

## General Instructions

The [Professor's package](https://github.com/CarmineD8/rt2_assignment1) has been modified by creating this repository, containing the given package and two additional branches:

1. `actions` 
2. `ros2`

Finally, in the main branch (or in one of the two added branches), I added a Vrep scene containing the robot interacting with the simulation. (You can choose if using ROS or ROS2 api)

## More about the "_action_" branch
This latter, should contain the same package in ROS, but with the **go_to_point** node modelled as a ROS action server, instead of a “simple” server.
Given that, **the robot FSM node** should now implement mechanisms for possibly cancelling the goal, when the related user command is received

## More about the "_ros2_" branch
In the branch `ros2`, the cpp nodes (Robot FSM and position server) should be written for ROS2, as **components**, so that, by using the _**ros1_bridge**_, they can be interface with the ROS nodes and with the simulation in Gazebo. **The go_to_point** can still be implemented as a service.
Also:

1. a _**launch file**_ to start the container manager and the components, should be created
2. a _**script**_ to launch all required nodes and the simulation should be implemented