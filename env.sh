#!/usr/bin/env bash

#cd /home/stero/ws_ids/devel/lib/limits     # cd to the directory with your node
#chown root:root limits # change ownship to root
#chmod a+rx limits      # set as executable by all
#chmod u+s limits       # set the setuid bit

source /home/stero/ws_ids/devel/setup.bash
export ROS_IP=192.168.1.122
export ROS_MASTER_URI=http://192.168.1.241:11311/
exec "$@"
