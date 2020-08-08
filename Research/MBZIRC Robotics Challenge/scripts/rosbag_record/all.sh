#!/bin/bash

echo "Recording to bag with prefix:" $1

rosbag record -o $1 -a
