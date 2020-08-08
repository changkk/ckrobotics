#1/bin/bash

ns=$1

rosservice call $ns"/mavros/set_stream_rate" 0 30 0
rosservice call $ns"/mavros/set_stream_rate" 3 15 1
rosservice call $ns"/mavros/set_stream_rate" 6 30 1
rosservice call $ns"/mavros/set_stream_rate" 10 30 1
rosservice call $ns"/mavros/set_stream_rate" 12 20 1
