#!/bin/bash

## Usage: 
# ./workstation.sh 1 "$(rospack find box)/maps/maze_00.yaml"

# Getting command line args
arg_shared_dir=$HOME/Public
arg_num_robots=$1
map_file=$2

# Running launch file
roslaunch box box_manage.launch arg_shared_dir:=$arg_shared_dir arg_num_robots:=$arg_num_robots map_file:=$map_file 


