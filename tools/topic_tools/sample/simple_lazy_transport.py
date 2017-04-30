#!/usr/bin/env python

import roslib.message
import rospy

from topic_tools import LazyTransport


class SimpleLazyTransport(LazyTransport):
    def __init__(self):
        super(self.__class__, self).__init__()
        msg_name = rospy.get_param('~msg_name')
        self.msg_class = roslib.message.get_message_class(msg_name)
        self._pub = self.advertise('~output', self.msg_class, queue_size=1)

    def subscribe(self):
        self._sub = rospy.Subscriber('~input', self.msg_class, self._process)

    def unsubscribe(self):
        self._sub.unregister()

    def _process(self, img_msg):
        self._pub.publish(img_msg)


if __name__ == '__main__':
    rospy.init_node('simple_transport')
    app = SimpleLazyTransport()
    rospy.spin()
