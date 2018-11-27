# Unix Domain Socket feature - Read me

## Index
- [1. Background/Overview](#1-backgroundoverview)
- [2. Unix Domain Socket(UDS) feature](#2-unix-domain-socketuds-feature)
  * [a) How to use in runtime](#a-how-to-use-in-runtime)
  * [b) Additional Sub Features](#b-additional-sub-features)
  * [c) Limitations](#c-limitations)
  * [d) Devel environment](#d-devel-environment)

## 1. Background/Overview

 this repository is to develop and distribute the Unix Domain Socket
 feature extension for ROS message transmission layer. if the system
 internal message use only, TCP/IP, UDP/IP protocal stack does not do
 any good to the system, just a pure overhead. so then Unix Domain
 Socket (UDS) is reasonable to get rid of the overhead with the system
 internal use for ROS Inter-Process Communication.

## 2. Unix Domain Socket(UDS) feature

### a. How to use in runtime

- for runtime, UDS feature is provided as BAS package for projects.
  1st of all, you do need to install the BAS package as well.
- TCP/IP and UDS feature are supported in hybrid mode.
  UDS feature will be used automaticlly, you can just use ROS as usual.
```console
    e.g) # roscore
         # rosrun roscpp_tutorials talker
         # rosrun roscpp_tutorials listener
```

### b. Additional Sub Features

- using UDS, there are some additional sub features.
  environment variable 'ROS_UDS_EXT_FEATURE=<Hexadecimal>'
  you can refer to these definition as followings.
```
    ROS_UDS_EXT_ABSTRACT_SOCK_NAME   0x00000001 /*!< enable abstract named socket */
```
  set environment variable 'ROS_UDS_EXT_FEATURE=<Hexadecimal>' to turn on each.
```console
    e.g) # export ROS_UDS_EXT_FEATURE=0x00000001
```
- environment variable 'ROS_NO_ROSOUT=1' </br>
  if you dont need to create rosout thread for each node. enable this.
```console
    e.g) # export ROS_NO_ROSOUT=1
```

### c. Limitations

the hybrid system is not supported if rosmaster is constructed via
ros_comm original(means it does not know Unix Domain Socket).

### d. Devel environment

you can use this repository in your catkin environment, all you need to
do is the following.
```console
    # cd <CATKIN_WORkSPACE>/src
    # git clone <repository>
    # cd <CATKIN_WORkSPACE>
    # catkin_make
    # source devel/setup.bash
```
