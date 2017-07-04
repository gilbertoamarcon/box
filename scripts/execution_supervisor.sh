#!/bin/bash

## Usage: 
# ./execution_supervisor.sh gil@ovunc 0

# Getting command line args
shared_dir=$HOME/Public
remote_server=$1
robot_id=$2

# Mounting shared folder
rm $shared_dir/* > /dev/null 2>&1
sshfs $remote_server:$shared_dir $shared_dir

# Running launch file
roslaunch box supervise.launch shared_dir:=$shared_dir robot_id:=$robot_id 


