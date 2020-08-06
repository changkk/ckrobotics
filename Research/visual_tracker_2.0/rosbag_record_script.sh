#!/bin/bash


echo Recording selected topics

#xterm -hold -e 'roslaunch libuvc_camera uvc_cam_air360.launch' &

rosbag record -o iucrc_vis_track /omnicam/image_raw/compressed /mavros/imu/data /mavros/local_position/pose /piksi_multi_base_station/gps_time /camera/image_raw/compressed /gimbal_imu_angles /gimbal_target_orientation /range /zoom /img_out/compressed /yolo_detection_box /gimbal_target_speed /tf /tf_static /piksi_multi_base_station/navsatfix_best_fix


sleep 1
echo Done recording, change bag name appropriately.

exit 0
