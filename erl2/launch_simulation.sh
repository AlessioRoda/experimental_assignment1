#!/bin/bash

gnome-terminal --tab --title="robot_simulation" -- bash -c "roslaunch erl2 assignment.launch 2</dev/null"
gnome-terminal --tab --title="armor" -- bash -c "sleep 1; rosrun armor execute it.emarolab.armor.ARMORMainService"
gnome-terminal --tab --title="init_arm" -- bash -c "sleep 3; rosrun erl2 init 2</dev/null"
gnome-terminal --tab --title="nodes" -- bash -c "sleep 5; roslaunch erl2 launch_nodes.launch"
gnome-terminal --tab --title="planning" -- bash -c "sleep 5; roslaunch erl2 planning.launch"