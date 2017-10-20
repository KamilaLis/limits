#!/usr/bin/env bash

source /home/stero/ws_ids/devel/setup.bash
export ROS_IP=192.168.1.122
export ROS_MASTER_URI=http://192.168.1.122:11311/
exec "$@"