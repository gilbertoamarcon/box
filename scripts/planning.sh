#!/bin/bash

# Backing up present working directory
hdir=$(pwd)

# Input renaming 
problem=$1
solution=$2

cd ~/dev/imapc/logistics/
./main.sh ~/Documents/prob $problem -push -s ~/Documents/stats.csv -p popf2 -pa

cp ~/Documents/prob/solution $solution

# Back to original working directory
cd $hdir



