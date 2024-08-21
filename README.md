# detic_ros

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/detic_ros)

Docker environment for Detic ROS driver. This repository was inspired by [HiroIshida/detic_ros](https://github.com/HiroIshida/detic_ros).

## Features

- ROS Noetic package for [Facebook Detic](https://github.com/facebookresearch/Detic)  
- Docker environment to execute the detic with [Microsoft Azure Kinect](https://azure.microsoft.com/en-us/products/kinect-dk/#overview)  
- Docker environment to execute the detic with [Intel RealSense D435](https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html) (or UVC camera)  

## Docker build environment (tested)

- [Ubuntu 20.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=20.04+LTS) (22.04 is not supported)
  - [ROS Noetic (Python3)](https://wiki.ros.org/noetic/Installation/Ubuntu)
  - Docker 20.10.22
  - Docker Compose v2.4.1
  - nvidia-docker2 2.11.0-1

## Installation
```bash
sudo apt install graphicsmagick-imagemagick-compat byobu -y && git clone git@github.com:Osaka-University-Harada-Laboratory/detic_ros.git --recursive --depth 1 && cd detic_ros/docker && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

## Usage

1. Connect the camera device to the computer with a USB3.0 cable

2. Execute the below command according to the sensing device connected
    ```bash
    cd detic_ros/docker && docker compose up
    ```

    ```bash
    ./utils/k4a_demo.sh
    ```

    ```bash
    ./utils/rsd435_demo.sh
    ```

    ```bash
    ./utils/webcam_demo.sh
    ```  
    <img src=image/demo.gif width=720>  

### Detic

#### Azure Kinect
  ```bash
  xhost + && docker exec -it detic_1804_container bash -it -c "roslaunch detic_ros k4a_bringup.launch"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros resize.launch"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros sample.launch out_debug_img:=true out_debug_segimg:=false compressed:=false device:=auto input_image:=/resized_image_color"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rosrun image_view image_view image:=/docker/detic_segmentor/debug_image"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rosrun image_view image_view image:=/docker/detic_segmentor/debug_segmentation_image _do_dynamic_scaling:=true"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rostopic echo /docker/detic_segmentor/segmentation_info/detected_classes"
  ```

#### RealSense D435
  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros rsd435_bringup.launch"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros sample.launch out_debug_img:=true out_debug_segimg:=false compressed:=false device:=auto input_image:=/camera/color/image_raw"
  ```  
  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rosrun image_view image_view image:=/docker/detic_segmentor/debug_image"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rosrun image_view image_view image:=/docker/detic_segmentor/debug_segmentation_image _do_dynamic_scaling:=true"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rostopic echo /docker/detic_segmentor/segmentation_info/detected_classes"
  ```

#### USB camera
  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros usbcam_bringup.launch" 
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "roslaunch detic_ros sample.launch out_debug_img:=true out_debug_segimg:=false compressed:=false device:=auto input_image:=/usb_cam/image_raw"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rosrun image_view image_view image:=/docker/detic_segmentor/debug_image"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rosrun image_view image_view image:=/docker/detic_segmentor/debug_segmentation_image _do_dynamic_scaling:=true"
  ```  

  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "rostopic echo /docker/detic_segmentor/segmentation_info/detected_classes"
  ```

### Other examples
#### Single image 
  ```bash
  xhost + && docker exec -it detic_2004_container bash -it -c "cd /catkin_ws/src/Detic && python3 demo.py --config-file configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml --input ../detic_ros/data/(image_name).png --output /tmp/out.png --opts MODEL.WEIGHTS models/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth"
  ```  

  ```bash
  docker cp detic_2004_container:/tmp/out.png /tmp/out.png && display /tmp/out.png
  ```

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Masato Tsuru](https://tsurumasato.github.io/)

We always welcome collaborators!

## License

Please refer to each package.xml of the ROS packages
