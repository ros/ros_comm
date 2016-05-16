#!/usr/bin/env python

import rospy
import rosgraph_msgs

class Rosout:
    'The rosout node subscribes to /rosout, logs the messages to file, and re-broadcasts the messages to /rosout_agg'
    def __init__(self):
        rospy.init_node('rosoutpy')
        self.pub = rospy.Publisher("/rosout_agg", rosgraph_msgs.msg.Log, queue_size=0)
        self.sub = rospy.Subscriber("/rosout", rosgraph_msgs.msg.Log, self.rosoutCallback, queue_size=1)

    def rosoutCallback(self, msg):
        self.pub.publish(msg)

    # Main function.
if __name__ == '__main__':
    try:
        rosout = Rosout()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
