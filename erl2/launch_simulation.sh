#!/bin/bash

gnome-terminal --tab --title="robot_simulation" -- bash -c "roslaunch erl2 assignment.launch 2</dev/null"
gnome-terminal --tab --title="nodes" -- bash -c "sleep 7; roslaunch erl2 launch_nodes.launch 2</dev/null"
gnome-terminal --tab --title="planning" -- bash -c "sleep 5; roslaunch erl2 planning.launch 2</dev/null"