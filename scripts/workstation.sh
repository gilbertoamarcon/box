#!/bin/bash

## Usage: 
# ./workstation.sh $HOME/Public 1

# Getting command line args
arg_shared_dir=$1
arg_num_robots=$2

# Cleaning shared folder
rm -Rf $arg_shared_dir/*

# Running launch file
roslaunch box box_manage.launch arg_shared_dir:=$arg_shared_dir arg_num_robots:=$arg_num_robots 


