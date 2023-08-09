#!/bin/bash

byobu new-session -d -s usbcam_demo
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 2

byobu send-keys -t 0 'xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros usbcam_bringup.launch"' 'C-m'
sleep 8.
byobu send-keys -t 1 'xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros sample.launch out_debug_img:=true out_debug_segimg:=false compressed:=false device:=auto input_image:=/usb_cam/image_raw"' 'C-m'
sleep 8.
byobu send-keys -t 2 'xhost + && docker exec -it detic_2004_container bash -it -c "rosrun rviz rviz -d /catkin_ws/src/detic_ros/config/usbcam_demo_rviz.rviz"' 'C-m'
sleep 1.

byobu attach -t usbcam_demo
