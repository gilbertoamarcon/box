#!/bin/bash

## Usage: 
# ./robot.sh gil@deskhp $HOME/Public 0

# Getting command line args
remote_server=$1
arg_shared_dir=$2
arg_robot_id=$3

# Mounting shared folder
# sshfs $remote_server:$arg_shared_dir $arg_shared_dir

# Running launch file
roslaunch box box_act.launch arg_shared_dir:=$arg_shared_dir arg_robot_id:=$arg_robot_id 


