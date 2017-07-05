# box

This package contains a multi-robot planning system for box pushing.
It will create a waypoint abstraction of the world and use centralized planning algorithms to push boxes to their goal destination.
There are two planning approaches: 
* An internal C++ optimal planner based on A*.
* An exterminal PDDL-compatible general planner interface.

Using the external planner requires installing and configuring the external planner, and creating the appropriate script under /scripts.



Basic Install: cd into models and:

```create-all.sh```

Run catkin_make at the workspace root.


Example Usage (Gazebo): 

On the "Robot 0" machine:

```export ROBOT_INITIAL_POSE="-y 0.5"```

```roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find box)/worlds/maze_01.world```

```roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find box)/maps/maze_01.yaml initial_pose_y:=0.5```

```rosrun box execution_supervisor.sh gil@deskhp 0```


On the "Robot 1" machine:

```export ROBOT_INITIAL_POSE="-y -0.5"```

```roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find box)/worlds/maze_01.world```

```roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find box)/maps/maze_01.yaml initial_pose_y:=-0.5```

```rosrun box execution_supervisor.sh gil@deskhp 1```


On the "Planner" machine:

```rosrun box execution_commander.sh 2 "$(rospack find box)/maps/maze_01.yaml"```

Example box start positions:

```1183 1047 876```

Example box end positions:

```1225 1219 707```


Example Usage (Real World): 

On the "Robot 0" machine:

```roslaunch turtlebot_bringup minimal.launch```

```roslaunch turtlebot_navigation amcl_demo.launch map_file:=$(rospack find box)/maps/printer_000.yaml```

```rosrun box execution_supervisor.sh gil@deskhp 0```


On the "Robot 1" machine:

```roslaunch turtlebot_bringup minimal.launch```

```roslaunch turtlebot_navigation amcl_demo.launch map_file:=$(rospack find box)/maps/printer_000.yaml```

```rosrun box execution_supervisor.sh gil@deskhp 1```


On the "Planner" machine:

```rosrun box execution_commander.sh 2 "$(rospack find box)/maps/printer_000.yaml"```

Example box start positions:

```1183 1047 876```

Example box end positions:

```1225 1219 707```

