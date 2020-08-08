#!/bin/bash

rostopic pub /hexacopter/uav_control/velocity geometry_msgs/Twist '{ linear: { x: 0.0, y: 0.5, z: 0.1 } }'
sleep 20
rostopic pub /hexacopter/uav_control/rtl std_msgs/Empty
