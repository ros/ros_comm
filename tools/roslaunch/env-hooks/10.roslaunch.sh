# roslaunch/env-hooks/10.roslaunch.sh

if [ ! "$ROS_MASTER_URI" ]; then
  export ROS_MASTER_URI=http://localhost:11311
fi
