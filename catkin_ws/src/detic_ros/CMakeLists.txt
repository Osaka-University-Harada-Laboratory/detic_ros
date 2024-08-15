cmake_minimum_required(VERSION 3.0.2)
project(detic_ros)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
    message_generation
)

add_message_files(
    FILES
    SegmentationInfo.msg
)

add_service_files(
    FILES
    DeticSeg.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    visualization_msgs
    message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
                      scripts/client.py
                      scripts/masked_image_publisher.py
                      scripts/wrapper.py
                      scripts/node.py
                      scripts/node_config.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
