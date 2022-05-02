First build the workspace: 

- colcon build


In order to start the turtlesim node, type in a new terminal:

- ros2 run turtlesim turtlesim_node


Then in another terminal source the workspace and run the modified controller node:

- source ~/dev_ws/install/setup.bash
- ros2 run usi_angry_turtle move2goal_node 


In order to spawn and control an offender turtle, execute the following commands in another terminal:

- ros2 service call /spawn turtlesim/srv/Spawn "{x: 10.5, y: 10.5, theta: 0.0, name: 'turtle2'}"
- ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/turtle2/cmd_vel

Note: no other offender turtle is spawnable unless there are no offender turtles already, or until an already existing one is killed. 