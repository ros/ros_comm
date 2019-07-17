#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2019, Yuki Furuta
# All rights reserved.
# Author: Yuki Furuta <me@furushchev.ru>

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Trigger, TriggerResponse


def empty_cb(req):
    return EmptyResponse()

def set_bool_cb(req):
    return SetBoolResponse()

def trigger_cb(req):
    return TriggerResponse()

def main():
    rospy.init_node('service_server')

    empty_service = rospy.Service('empty', Empty, empty_cb)
    set_bool_service = rospy.Service('set_bool', SetBool, set_bool_cb)
    trigger_service = rospy.Service('trigger', Trigger, trigger_cb)

    rospy.spin()


if __name__ == '__main__':
    main()
