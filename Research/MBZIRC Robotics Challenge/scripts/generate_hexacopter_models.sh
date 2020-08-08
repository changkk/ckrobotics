#!/bin/bash

if [ "$MBZIRC_DIR" == "" ]
then
  echo "You need to set the MBZIRC_DIR variable for this script."
  exit
fi

MODEL_DIR="$MBZIRC_DIR/src/mbzirc_gazebo/models/hexacopter"
CONFIG_DIR="$MODEL_DIR/config"

UDP_PORT_OFFSET=100

UDP_PORT_MAVROS_NODE=14540
UDP_PORT_MAVLINK_MAIN=14556
UDP_PORT_SIMULATOR_AUTOPILOT=14557
UDP_PORT_SIMULATOR_MAVLINK=14560

TEMPLATE_MODEL_SDF="$MODEL_DIR/model.sdf.erb"
TEMPLATE_MODEL_CONFIG="$CONFIG_DIR/hexacopter.erb"

for i in `seq 0 3`;
do
  MODEL_TAG=""
  MAGNETIC_GRIPPER="false"
  if [ $i -gt 0 ]
  then
    MODEL_TAG=$(($i - 1))
    MAGNETIC_GRIPPER="true"
    UDP_PORT_MAVROS_NODE=$((UDP_PORT_MAVROS_NODE + UDP_PORT_OFFSET))
    UDP_PORT_MAVLINK_MAIN=$((UDP_PORT_MAVLINK_MAIN + UDP_PORT_OFFSET))
    UDP_PORT_SIMULATOR_AUTOPILOT=$((UDP_PORT_SIMULATOR_AUTOPILOT + UDP_PORT_OFFSET))
    UDP_PORT_SIMULATOR_MAVLINK=$((UDP_PORT_SIMULATOR_MAVLINK + UDP_PORT_OFFSET))
  fi

  MODEL_SDF="$MODEL_DIR/model$MODEL_TAG.sdf"
  MODEL_CONFIG="$CONFIG_DIR/hexacopter$MODEL_TAG"

  erb \
    model_tag="$MODEL_TAG" \
    magnetic_gripper="$MAGNETIC_GRIPPER" \
    simulator_mavlink_udp_port=$UDP_PORT_SIMULATOR_MAVLINK "$TEMPLATE_MODEL_SDF" > "$MODEL_SDF"
  erb \
    mavros_node_udp_port=$UDP_PORT_MAVROS_NODE \
    mavlink_main_udp_port=$UDP_PORT_MAVLINK_MAIN \
    simulator_autopilot_udp_port=$UDP_PORT_SIMULATOR_AUTOPILOT \
    simulator_mavlink_udp_port=$UDP_PORT_SIMULATOR_MAVLINK \
    "$TEMPLATE_MODEL_CONFIG" > "$MODEL_CONFIG"
done
