# Speedier lookups with rosbag info -y --key

## Overview
Project was based on issue reported here <https://github.com/ros/ros_comm/issues/944>


The tool is working slowly due to frequent calls on hard drive disk. This conclusion was made according to results of several tests using linux *time* command which reported that real time was much greater than user and sys time taken together. So the main goal was to minimize a number of hdd calls.

As the issue reports problems only with running tool with specified key, an obvious solution was only to fetch the information required for this key and do not read the file completely. Although it gave more than satisfactory results on keys *path*, *size*, *version*, *topics* and so on speeding it up almost by hundred times, it still takes a long time to run it with key *compression* as it requires reading *Chunk Headers*.

The original tool was implemented in python <https://github.com/ros/ros_comm/tree/lunar-devel/tools/rosbag/src/rosbag>, my implementation is based on existing code for parsing bag files <https://github.com/ros/ros_comm/blob/lunar-devel/tools/rosbag_storage/src/bag.cpp> to reduce number of possible errors. 
If rosbag info is launched with keys -y -k, it executes my code by calling executable file (you have build it before).

## Build

To build the executable file you need to open directory *ros_comm/tools/rosbag/rosbag_info_pkg*, call *cmake CMakeLists.txt* and *make*.
The executable file will be put in */devel/lib/rosbag_info_pkg*.


## Tests

The easiest way to test this code was to download the existing file and compare output of the original and my own tool. To simplify this process I write the two bash scripts.  
First script downloads the file and calls the second one with different parameters. The second script executes both original and newly developed programs, writes the results in two temporary files gets the real work time, compares results and computes the ratio between work times. The first script prints average ratio.
The memory is cached so we need to clear cache after every execution to compare time correctly.
To run first script you have to do the following:
chmod +x all_tests.sh
./all_tests.sh
enter the password for sudo 

I have chosen mid-sized file as an example in order to reduce exection time and required memory.
We take note that the ratio between work times of original and newly developed grows lineary with increase of file size. For example, the average on tested 10G
file is about 150.
   

