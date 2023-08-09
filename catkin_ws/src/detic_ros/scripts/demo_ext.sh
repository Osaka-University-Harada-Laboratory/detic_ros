#!/bin/bash

for i in "01" "02" "03" "04" "05" "06" "07" "08" "09" "10" "11" "12" "13" "14" "15" "16" "17" "18" "19" "20" "21" "22" "23" "24" "25" "26" "27" "28" "29" "30" "31" "32" "33" "34" "35" "36" "37" "38" "39" "40" "41" "42" "43" "44" "45" "46" "47" "48" "49" "50"
do

    xhost + && docker exec -it detic_2004_container bash -it -c "cd /catkin_ws/src/Detic && python3 demo_ext.py --config-file configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml --input ../detic_ros/data/snacks/image${i}.png --output /tmp/out${i}.png --opts MODEL.WEIGHTS models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth"

    docker cp detic_2004_container:/tmp/out${i}.png /tmp/out$i.png
    docker cp detic_2004_container:/tmp/out${i}_region.png /tmp/out_region$i.png
    docker cp detic_2004_container:/tmp/out${i}_region_center.png /tmp/out_region_center$i.png
done
