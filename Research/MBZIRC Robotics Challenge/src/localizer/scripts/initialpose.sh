#!/bin/bash

sleep 1

rostopic pub -1f $(rospack find localizer)/scripts/initialpose.yaml /initialpose geometry_msgs/PoseWithCovarianceStamped
