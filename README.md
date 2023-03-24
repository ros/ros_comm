# Leggedrobotics ROS Comm

In case you are experiencing high latency and delay in a Python ROS subscriber used for real-time applications, we might have the solution for you. In many of our applications we experienced an increasing latency, despite having a queue of 1 selected. This goes back to [this](https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/?answer=220505?answer=220505#post-id-220505) discussion.
Clone this repository on branch `dev` into your workspace and build your package as regular. As your package should have a run dependency on `rospy`, `ros_comm` and `rospy` should also automatically be built.

After sourcing of your workspace, the latency should go down to the actual time spent in the callback, instead of accumulating delay over time.

We in particular encountered this issue when having computationally-heavy code running inside the callback, such as neural network inference or similar.
