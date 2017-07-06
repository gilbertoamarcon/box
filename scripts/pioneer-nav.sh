#!/bin/bash

echo "Launching roscore..."
roscore &
pid=$!
sleep 5s

echo "Connecting to RosAria..."
rosrun rosaria RosAria &
pid="$! $pid"
sleep 3s

echo "Launching sensors..."
roslaunch pioneer_test pioneer_sensors.launch &
pid="$! $pid"

sleep 3s

echo "Launching navigation stack..."
roslaunch box nav_bundle.launch map_name:=$(rospack find box)/maps/printer_000.yaml &
pid="$! $pid"

sleep 3s

echo "Launching pioneer controller..."
roslaunch pioneer_test pioneer_controller_spin_recover.launch &
pid="$! $pid"

sleep 3s

echo "Launching rviz..."
roslaunch pioneer_test pioneer_description.launch &
rosrun rviz rviz -d "$(rospack find box)/rviz/pioneer.rviz" &
pid="$! $pid"

sleep 1s

#echo "Launching estop..."
#rqt --standalone rqt_estop

sleep 1s

trap "echo Killing all processes.; kill -2 $pid; exit" SIGINT SIGTERM

sleep 24h
