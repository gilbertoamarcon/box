#!/bin/bash

## Usage: 
# ./workstation.sh 1

# Getting command line args
arg_shared_dir=$HOME/Public
arg_num_robots=$1

# Cleaning shared folder
rm $arg_shared_dir/* > /dev/null 2>&1

# Running launch file
roslaunch box box_manage.launch arg_shared_dir:=$arg_shared_dir arg_num_robots:=$arg_num_robots 


