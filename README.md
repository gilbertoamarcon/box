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


Usage: 

On the "Robot" machine:

```roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find box)/worlds/maze_00.world```

```roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find box)/maps/maze_00.yaml```

```rosrun box execution_supervisor.sh gil@ovunc 0```


On the "Planner" machine:

```rosrun box execution_commander.sh 1 "$(rospack find box)/maps/maze_00.yaml"```

