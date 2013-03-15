REM roslaunch/env-hooks/10.roslaunch.bat

if [%ROS_MASTER_URI%]==[] (
  set ROS_MASTER_URI=http://localhost:11311
)
