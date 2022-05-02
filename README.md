## Description

Robotics assignment realized for Università della Svizzera Italiana. 

It's a turtlesim controller node for ROS2: one turtle is tasked with writing something (in this case: "USI") and another spawnable and controllable turtle is an offender. If the offender gets too close to the writing turtle, this becomes angry and starts chasing the offender until it kills it. The writing turtle gets then back to its writing behaviour. 

__N.B. Requires an installation of ROS2 in order to work.__

## Instructions to run

First place the package inside the workspace source folder, for example: *~/<workspace_folder>/src/* 


Build the workspace: 

    colcon build


In order to start the turtlesim node, type in a new terminal:

    ros2 run turtlesim turtlesim_node


Then in another terminal source the workspace and run the modified controller node:

    source ~/<workspace_folder>/install/setup.bash

    ros2 run usi_angry_turtle move2goal_node 


In order to spawn and control an offender turtle, execute the following commands in another terminal:

    ros2 service call /spawn turtlesim/srv/Spawn "{x: 10.5, y: 10.5, theta: 0.0, name: 'turtle2'}"

    ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/turtle2/cmd_vel

Note: in the current version, no other offender turtle is spawnable unless there are no offender turtles already, or until an already existing one is killed. 