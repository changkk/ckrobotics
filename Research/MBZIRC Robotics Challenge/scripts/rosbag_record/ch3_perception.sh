#!/bin/bash

ns=$1
name=${2:-pid}
topics=""

topics+=$ns"/perspective_camera/image_raw "
topics+=$ns"/perspective_camera/camera_info "
topics+=$ns"/fisheye_camera/image_raw "
topics+=$ns"/fisheye_camera/camera_info "
topics+=$ns"/gps/fix "
topics+=$ns"/gps/rtkfix "
topics+=$ns"/gps/time "
topics+=$ns"/localizer/gps_imu_odom "
topics+=$ns"/localizer/global_odom "
topics+=$ns"/localizer/clog_pose "
topics+=$ns"/mavros/imu/data "
topics+=$ns"/mavros/rc/override "
topics+=$ns"/mavros/rc/in "
topics+=$ns"/mavros/local_position/odom "
topics+=$ns"/uav_control/waypoint "
topics+=$ns"/uav_control/waypoint/arrived "
topics+=$ns"/uav_control/velocity "
topics+=$ns"/uav_control/target/search "
topics+=$ns"/uav_control/target/acquire "
topics+=$ns"/magnetic_gripper/target_captured "
topics+=$ns"/perception/track_target/feedback "
topics+=$ns"/perception/targets "
topics+=$ns"/ocs/log_message "
topics+="/tf "
topics+="/tf_static "

echo "Recording bag. Topics: " $topics

rosbag record -o $name $topics
