#!/bin/bash

ns=$1
name=${2:-images}
topics=""

topics+=$ns"/perspective_camera/image_raw "
topics+=$ns"/perspective_camera/camera_info "
topics+=$ns"/fisheye_camera/image_raw "
topics+=$ns"/fisheye_camera/camera_info "
topics+="/tf "
topics+="/tf_static "

echo "Recording bag. Topics: " $topics

rosbag record -o $name $topics
