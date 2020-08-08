#!/bin/bash

if [ "$MBZIRC_DIR" == "" ]
then
  echo "You need to set the MBZIRC_DIR variable for this script."
  exit
fi

if [ "$SITL_GAZEBO_PATH" == "" ]
then
  echo "Configure environment variables; see README."
  exit
fi

echo 'Environment variables configured. Installing...'
sleep 1

# Build sitl_gazebo
SOURCE_DIR="$MBZIRC_DIR/src"
mkdir -p $SOURCE_DIR
cd $SOURCE_DIR
git clone --recursive https://github.com/PX4/sitl_gazebo.git sitl_gazebo
cd sitl_gazebo
mkdir build
cd build
cmake ..
make
