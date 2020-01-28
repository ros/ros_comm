REM roslaunch/env-hooks/10.roslaunch.bat

:: workaround Python 2 xmlrpc performance issue
:: https://stackoverflow.com/questions/2617615/slow-python-http-server-on-localhost
:: use IP address instead to avoid unnecessary DNS lookups.
if "%ROS_MASTER_URI%" == "" (
  set ROS_MASTER_URI=http://127.0.0.1:11311
)

if "%ROS_IP%" == "" (
  set ROS_IP=127.0.0.1
)
