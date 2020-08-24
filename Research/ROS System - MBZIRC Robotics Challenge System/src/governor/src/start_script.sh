#!/bin/bash



sleep 10

echo Changing mode to OFFBOARD.
rosservice call /hexacopter0/mavros/set_mode 64 OFFBOARD
rosservice call /hexacopter1/mavros/set_mode 64 OFFBOARD
rosservice call /hexacopter2/mavros/set_mode 64 OFFBOARD

sleep 1

echo Arming
rosservice call /hexacopter0/mavros/cmd/arming 1
rosservice call /hexacopter1/mavros/cmd/arming 1
rosservice call /hexacopter2/mavros/cmd/arming 1

sleep 2

echo Taking Off
rosservice call /hexacopter0/mavros/cmd/takeoff 1 0 47.3974191 8.5460728 493
rosservice call /hexacopter1/mavros/cmd/takeoff 1 0 47.3974191 8.5460728 495
rosservice call /hexacopter2/mavros/cmd/takeoff 1 0 47.3974191 8.5460728 497

sleep 4

echo Changing mode to Offboard again
rosservice call /hexacopter0/mavros/set_mode 64 OFFBOARD
rosservice call /hexacopter1/mavros/set_mode 64 OFFBOARD
rosservice call /hexacopter2/mavros/set_mode 64 OFFBOARD

rosrun governor psuedo_target_generator_publisher 










