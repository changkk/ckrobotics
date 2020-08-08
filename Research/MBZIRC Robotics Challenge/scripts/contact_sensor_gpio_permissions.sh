#!/bin/bash

if [ "$MBZIRC_DIR" == "" ]
then
  echo "You need to set the MBZIRC_DIR variable for this script."
  exit
fi

echo 'Setting permissions for GPIO pins...'


SOURCE_DIR="$MBZIRC_DIR/devel/lib/contact_sensor/"

cd $SOURCE_DIR

sudo chown root:root contact_node 

sudo chmod a+rx contact_node 

sudo chmod u+s contact_node


