#!/bin/bash

# Backing up present working directory
hdir=$(pwd)

# Input renaming 
problem=$1
solution=$2

cd ~/dev/imapc/scripts/
./synth-and-run.sh ${HOME}/Documents/prob $problem -push -so -cfa 'DynamicProgramming' -s ${HOME}/Documents/stats.csv -p popf2 -pa

cp ~/Documents/prob/solution $solution

# Back to original working directory
cd $hdir



