#!/bin/bash


echo Recording selected topics

#xterm -hold -e 'roslaunch libuvc_camera uvc_cam_air360.launch' &

rosbag record -o iucrc_vis_track /omnicam/image_raw/compressed /mavros/imu/data /mavros/global_position/local /mavros/global_position/global /camera/image_raw/compressed /gimbal_imu_angles /gimbal_target_orientation /zoom /img_out/compressed /gimbal_cue /yolo_detection_box /gimbal_target_speed /man_zoom 


sleep 1
echo Done recording, change bag name appropriately.

exit 0
