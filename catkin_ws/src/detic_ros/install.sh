#!/bin/bash

cd /catkin_ws/src/detic_ros && ln -sf ../Detic/configs detic_configs && ln -sf ../Detic/datasets datasets && ln -sf ../Detic/models models
cd /catkin_ws/src/detic_ros/scripts && ln -sf ../../Detic/detic && ln -sf ../../Detic/third_party