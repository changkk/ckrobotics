#!/bin/bash

ns=$1
name=${2:-ch1_perception}
topics=""

topics+=$ns"/perspective_camera/image_raw "
topics+=$ns"/perspective_camera/camera_info "
topics+=$ns"/uav_control/waypoint "
topics+=$ns"/rangefinder_raw "
topics+=$ns"/rangefinder "
topics+=$ns"/fisheye_camera/camera_info "
topics+=$ns"/fisheye_camera/image_raw "
topics+=$ns"/mavros/imu/data "
topics+=$ns"/mavros/local_position/odom "
topics+=$ns"/gps/fix "
topics+=$ns"/gps/rtkfix "
topics+=$ns"/gps/time "
topics+=$ns"/localizer/gps_imu_odom "
topics+=$ns"/localizer/global_odom "
topics+=$ns"/localizer/clog_pose "
topics+=$ns"/uav_control/velocity "
topics+=$ns"/uav_control/waypoint "
topics+=$ns"/mavlink/from "
topics+="/tf "
topics+="/tf_static "

echo "Recording bag. Topics: " $topics

rosbag record -o $name $topics
