#!/bin/bash

byobu new-session -d -s k4a_demo
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 2
byobu split-window -h
byobu select-pane -t 3

byobu send-keys -t 0 'xhost + && docker exec -it detic_1804_container bash -it -c "roslaunch detic_ros k4a_bringup.launch"' 'C-m'
sleep 6.
byobu send-keys -t 1 'xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros resize.launch"' 'C-m'
sleep 2.
byobu send-keys -t 2 'xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros sample.launch out_debug_img:=true out_debug_segimg:=false compressed:=false device:=auto input_image:=/resized_image_color"' 'C-m'
sleep 8.
byobu send-keys -t 3 'xhost + && docker exec -it detic_2004_container bash -it -c "rosrun rviz rviz -d /catkin_ws/src/detic_ros/config/k4a_demo_rviz.rviz"' 'C-m'
sleep 1.

byobu attach -t k4a_demo
