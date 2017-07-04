#!/bin/bash

## Usage: 
# ./robot.sh gil@ovunc 0

# Getting command line args
arg_shared_dir=$HOME/Public
remote_server=$1
arg_robot_id=$2

# Mounting shared folder
sshfs $remote_server:$arg_shared_dir $arg_shared_dir

# Running launch file
roslaunch box box_act.launch arg_shared_dir:=$arg_shared_dir arg_robot_id:=$arg_robot_id 


