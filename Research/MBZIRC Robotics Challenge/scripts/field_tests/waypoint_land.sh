#!/bin/bash

rostopic pub /hexacopter/uav_control/waypoint geometry_msgs/Pose '{ position: { x: 0.0, y: 5.0, z: 2.0 } }'

rostopic pub /hexacopter/uav_control/land std_msgs/Empty
