# Commands

X should be: -25 < x < -10 (5 meter buffer to arena edge on x)
Y should be: -35 < y < 35 (10 meter buffer to arena edge on y)

## Take control from pilot
    rostopic pub /hexacopter2/uav_control/resume_mission std_msgs/Empty

## Takeoff
    rostopic pub /hexacopter2/uav_control/resume_mission std_msgs/Empty
    rostopic pub /hexacopter2/uav_control/takeoff std_msgs/Empty

## Search pattern
    rostopic pub /hexacopter2/uav_control/target/search std_msgs/Empty

## Waypoint example
    rostopic pub /hexacopter2/uav_control/waypoint geometry_msgs/Pose '{ position: { x: -15.0, y: 0.0, z: 12.0 } }'

## Acquire target
    rostopic pub /hexacopter2/uav_control/waypoint geometry_msgs/Pose '{ position: { x: -15.0, y: 0.0, z: 12.0 } }'
    rostopic pub /hexacopter2/uav_control/target/acquire geometry_msgs/Pose '{ position: { x: -15.0, y: 0.0, z: 12.0 } }'

## Drop target
    rostopic pub /hexacopter2/uav_control/waypoint geometry_msgs/Pose '{ position: { x: -15.0, y: 0.0, z: 12.0 } }'
    rostopic pub /hexacopter2/uav_control/target/drop std_msgs/Empty

## Land at end of mission
1. RC - RTL flight mode switch
2. RC - extend landing gear switch

# Topics

    rostopic echo /hexacopter2/mavros/rc/in
    rostopic echo /hexacopter2/mavros/rc/override
    rostopic echo /hexacopter2/mavros/state
    rostopic echo /hexacopter2/uav_control/state
    rostopic echo /hexacopter2/uav_control/target/state
    rostopic echo /hexacopter2/perception/state
    rostopic echo /hexacopter2/localizer/global_odom
    rostopic echo /hexacopter2/contact_sensors
