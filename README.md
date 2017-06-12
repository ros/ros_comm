# ros_comm
ROS communications-related packages, including core client libraries (roscpp, rospy, roslisp) and graph introspection tools (rostopic, rosnode, rosservice, rosparam).


Rosbag: create an option to filter for topics published by a specific node(issue #819).

1) What's changed.

   I created two keys for separate commands:
       rosbag record --publisher=NODE - records all messages by topics subscribed to by a specific node. It is like a --node=NODE, but from other side.
       rosbag filter in.bag out.bag --node=NODE - filters for topics subscribed to by a specific node

2) Realization
    
    Basically, ros already provided with all the tools necessary to accomplish that.
    Realization of a first function required me to add new parameter in recorder and new option in record_cmd in rosbag_main.py
    After that I had to find a function that made a thing I wanted and basically copy-paste it in recorder.cpp
    
    Second function required me to create a new cpp-file and, therefore, to change build paths. Other than that, it was a task of writing a simple parser.

3) Build

    You need to replace your ./src/ros_comm/tools/rosbag with my ./src/ros_comm/tools/rosbag - nothing was changed aside of this folder.
    Next you need to execute:
        ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
    Now you're ready to test.
    
4) Testing

    To test these functions we'll use canonical turtlesim. Open four terminals, set up ros in every one of them.
    Now, on three of them execute these commands:
        roscore
        rosrun turtlesim turtlesim_node
        rosrun turtlesim teleop_turtle_key
    in the last one execute: 
        rosbag record -O test.bag --publisher=teleop_turtle.
    You'll see the names of topics subscribed to buy teleop_turtle. After that, if you execute 'rosbag play test.bag', you'll see the turtle repeat its movements.
    To test the second function:
        rosbag record -O test.bag --publisher=teleop_turtle.       
        rosbag filter test.bag test2.bag --node=teleop_turtle
        rosbag play test2.bag
