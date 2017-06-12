#! /bin/bash
EXPECTED=$(mktemp /tmp/expected.XXXXXXXX)
RESULT=$(mktemp /tmp/result.XXXXXXXX)
TIMEFORMAT=%R
if [ "$3" == "--freq" ];then 
   OLD_TIME=$(time (rosbag info -y -k "$1" --freq "$2" >  "$EXPECTED") 2>&1)
   #clear cached memory to calculate time correctly
   sudo sync; sudo bash -c "echo 3 > /proc/sys/vm/drop_caches"
   NEW_TIME=$(time (./devel/lib/rosbag_info_pkg/rosbag-info -y -k "$1" --freq "$2" > "$RESULT") 2>&1)
else 
   OLD_TIME=$(time (rosbag info -y -k "$1" "$2" >  "$EXPECTED") 2>&1)
   sudo sync; sudo bash -c "echo 3 > /proc/sys/vm/drop_caches"
   NEW_TIME=$(time (./devel/lib/rosbag_info_pkg/rosbag-info -y -k "$1" "$2" > "$RESULT") 2>&1)
fi
diff -B  "$EXPECTED" "$RESULT" >&2 #> /dev/null 2>&1 
if [ $? -eq 0 ];then
   echo "$1 $3" >> rosbag_info_log.txt
   echo "old time" >> rosbag_info_log.txt
   echo "$OLD_TIME" >> rosbag_info_log.txt
   echo "new time" >> rosbag_info_log.txt
   echo "$NEW_TIME" >> rosbag_info_log.txt
   echo "scale=2; $OLD_TIME / $NEW_TIME" | bc -l
else
   echo FAIL
   exit 1
fi
sudo sync; sudo bash -c "echo 3 > /proc/sys/vm/drop_caches"

