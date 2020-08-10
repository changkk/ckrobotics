#!/bin/bash

ns=$1
name=${2:-pid}
topics=""


topics+=$ns"/pid/x_position/state "
topics+=$ns"/pid/x_position/setpoint "
topics+=$ns"/pid/x_position/control_effort "
topics+=$ns"/pid/y_position/state "
topics+=$ns"/pid/y_position/setpoint "
topics+=$ns"/pid/y_position/control_effort "
topics+=$ns"/pid/z_position/state "
topics+=$ns"/pid/z_position/setpoint "
topics+=$ns"/pid/z_position/control_effort "
topics+=$ns"/localizer/global_odom "
topics+="/tf "
topics+="/tf_static "

echo "Recording bag. Topics: " $topics

rosbag record -o $name $topics
