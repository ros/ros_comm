# rosbag
This is a custom rosbag package which provides all the feature of [original rosbag](http://wiki.ros.org/rosbag/Commandline) with custom freq recording of topics.

## how to record custom rosbag
```shell
rosbag record --file-name record.yaml -a
```
> this only throttles down the topics mentioned in record.yaml, other topics will be recorded at their publish rate.
### param 
* --file-name: file which has topics for custom recording

## To-Do
* currently it does not detect topic to record from file (file is only used for freq control of specific topic which is being recorded)


## example:- record.yaml
```yaml
# topic_name: freq(must be an integer)
"/chatter": 2
"/raw_odom": 1
"/realsense_bottom/accel/sample": 1
"/realsense_bottom/depth/camera_info": 1
"/realsense_bottom/depth/color/points": 1
"/realsense_bottom/depth/image_rect_raw": 1
"/realsense_bottom/infra1/camera_info": 1
"/realsense_bottom/infra1/image_rect_raw": 1
```
