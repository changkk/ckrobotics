#!/bin/bash

if [ "$MBZIRC_DIR" == "" ]
then
  echo "You need to set the MBZIRC_DIR variable for this script."
  exit
fi

echo 'Environment variables configured. Updating...'
sleep 1

cd "$MBZIRC_DIR/src/px4_firmware"
git pull

cd "$MBZIRC_DIR"
wstool update -t src
