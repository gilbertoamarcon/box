#!/bin/bash

## Usage: 
# ./execution_commander.sh 1 "$(rospack find box)/maps/maze_00.yaml"

# Getting command line args
shared_dir=$HOME/Public
num_robots=$1
map_file=$2

# Running launch file
roslaunch box command_optimal.launch shared_dir:=$shared_dir num_robots:=$num_robots map_file:=$map_file 


