#!/bin/bash -xe
#
# Run bloom
# https://gist.github.com/awesomebytes/196eab972a94dd8fcdd69adfe3bd1152
rosdep update
source /etc/os-release
cd hal_hw_interface
bloom-generate rosdebian --os-name $ID --os-version $VERSION_CODENAME \
    --ros-distro noetic
