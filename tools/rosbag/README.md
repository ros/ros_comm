# rosbag
This is a custom rosbag package which provides all the feature of [original rosbag](http://wiki.ros.org/rosbag/Commandline) with custom freq recording of topics.

## dependency
* Yaml-CPP :- https://github.com/jbeder/yaml-cpp

## how to record custom rosbag
```shell
rosbag record --file-name record.yaml
```
> this only records and throttles down the topics mentioned in record.yaml
## param 
* --file-name: file which has topics for custom recording

## example:- record.yaml
```yaml
# topic_name: freq(double)
# topic_name: -1 (it will be recorded at publish freq)
"/chatter": 2
"/raw_odom": 1
"/realsense_bottom/accel/sample": 1
"/realsense_bottom/depth/camera_info": 1
"/realsense_bottom/depth/color/points": 1
"/realsense_bottom/depth/image_rect_raw": 1
"/realsense_bottom/infra1/camera_info": 1
"/realsense_bottom/infra1/image_rect_raw": 1
```
