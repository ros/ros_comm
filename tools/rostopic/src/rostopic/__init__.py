#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# make sure we aren't using floor division
from __future__ import division, print_function

NAME='rostopic'

import argparse
import os
import sys
import math
import socket
import time
import traceback
import yaml
try:
    from xmlrpc.client import Fault
except ImportError:
    from xmlrpclib import Fault

from operator import itemgetter
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

import genpy

import roslib.message
import rosgraph
#TODO: lazy-import rospy or move rospy-dependent routines to separate location
import rospy

try:
    long
except NameError:
    long = int


class ROSTopicException(Exception):
    """
    Base exception class of rostopic-related errors
    """
    pass
class ROSTopicIOException(ROSTopicException):
    """
    rostopic errors related to network I/O failures
    """
    pass

def _check_master():
    """
    Make sure that master is available
    :raises: :exc:`ROSTopicException` If unable to successfully communicate with master
    """
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")
    
def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

class ROSTopicHz(object):
    """
    ROSTopicHz receives messages for a topic and computes frequency stats
    """
    def __init__(self, window_size, filter_expr=None, use_wtime=False):
        import threading
        from collections import defaultdict
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.times = []
        self._last_printed_tn = defaultdict(int)
        self._msg_t0 = defaultdict(lambda: -1)
        self._msg_tn = defaultdict(int)
        self._times = defaultdict(list)
        self.filter_expr = filter_expr
        self.use_wtime = use_wtime
        
        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

    def get_last_printed_tn(self, topic=None):
        if topic is None:
            return self.last_printed_tn
        return self._last_printed_tn[topic]

    def set_last_printed_tn(self, value, topic=None):
        if topic is None:
            self.last_printed_tn = value
        self._last_printed_tn[topic] = value

    def get_msg_t0(self, topic=None):
        if topic is None:
            return self.msg_t0
        return self._msg_t0[topic]

    def set_msg_t0(self, value, topic=None):
        if topic is None:
            self.msg_t0 = value
        self._msg_t0[topic] = value

    def get_msg_tn(self, topic=None):
        if topic is None:
            return self.msg_tn
        return self._msg_tn[topic]

    def set_msg_tn(self, value, topic=None):
        if topic is None:
            self.msg_tn = value
        self._msg_tn[topic] = value

    def get_times(self, topic=None):
        if topic is None:
            return self.times
        return self._times[topic]

    def set_times(self, value, topic=None):
        if topic is None:
            self.times = value
        self._times[topic] = value

    def callback_hz(self, m, topic=None):
        """
        ros sub callback
        :param m: Message instance
        :param topic: Topic name
        """
        # #694: ignore messages that don't match filter
        if self.filter_expr is not None and not self.filter_expr(m):
            return
        with self.lock:
            curr_rostime = rospy.get_rostime() if not self.use_wtime else \
                    rospy.Time.from_sec(time.time())

            # time reset
            if curr_rostime.is_zero():
                if len(self.get_times(topic=topic)) > 0:
                    print("time has reset, resetting counters")
                    self.set_times([], topic=topic)
                return
            
            curr = curr_rostime.to_sec() if not self.use_wtime else \
                    rospy.Time.from_sec(time.time()).to_sec()
            if self.get_msg_t0(topic=topic) < 0 or self.get_msg_t0(topic=topic) > curr:
                self.set_msg_t0(curr, topic=topic)
                self.set_msg_tn(curr, topic=topic)
                self.set_times([], topic=topic)
            else:
                self.get_times(topic=topic).append(curr - self.get_msg_tn(topic=topic))
                self.set_msg_tn(curr, topic=topic)

            #only keep statistics for the last 10000 messages so as not to run out of memory
            if len(self.get_times(topic=topic)) > self.window_size - 1:
                self.get_times(topic=topic).pop(0)

    def get_hz(self, topic=None):
        """
        calculate the average publising rate

        @returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if not self.get_times(topic=topic):
            return
        elif self.get_msg_tn(topic=topic) == self.get_last_printed_tn(topic=topic):
            return
        with self.lock:
            #frequency
            
            # kwc: In the past, the rate decayed when a publisher
            # dies.  Now, we use the last received message to perform
            # the calculation.  This change was made because we now
            # report a count and keep track of last_printed_tn.  This
            # makes it easier for users to see when a publisher dies,
            # so the decay is no longer necessary.
            
            n = len(self.get_times(topic=topic))
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.get_times(topic=topic)) / n
            rate = 1./mean if mean > 0. else 0

            #std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.get_times(topic=topic)) /n)

            # min and max
            max_delta = max(self.get_times(topic=topic))
            min_delta = min(self.get_times(topic=topic))

            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)

        return rate, min_delta, max_delta, std_dev, n+1

    def print_hz(self, topics=(None,)):
        """
        print the average publishing rate to screen
        """
        if len(topics) == 1:
            ret = self.get_hz(topics[0])
            if ret is None:
                print("no new messages")
                return
            rate, min_delta, max_delta, std_dev, window = ret
            print("average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(rate, min_delta, max_delta, std_dev, window))
            return

        # monitoring multiple topics' hz
        header = ['topic', 'rate', 'min_delta', 'max_delta', 'std_dev', 'window']
        stats = {h: [] for h in header}
        for topic in topics:
            hz_stat = self.get_hz(topic)
            if hz_stat is None:
                continue
            rate, min_delta, max_delta, std_dev, window = hz_stat
            stats['window'].append(str(window))
            stats['topic'].append(topic)
            stats['rate'].append('{:.4}'.format(rate))
            stats['min_delta'].append('{:.4}'.format(min_delta))
            stats['max_delta'].append('{:.4}'.format(max_delta))
            stats['std_dev'].append('{:.4}'.format(std_dev))
            stats['window'].append(str(window))
        if not stats['topic']:
            print('no new messages')
            return
        print(_get_ascii_table(header, stats))

def _get_ascii_table(header, cols):
    # compose table with left alignment
    header_aligned = []
    col_widths = []
    for h in header:
        col_width = max(len(h), max(len(el) for el in cols[h]))
        col_widths.append(col_width)
        header_aligned.append(h.center(col_width))
        for i, el in enumerate(cols[h]):
            cols[h][i] = str(cols[h][i]).ljust(col_width)
    # sum of col and each 3 spaces width
    table_width = sum(col_widths) + 3 * (len(header) - 1)
    n_rows = len(cols[header[0]])
    body = '\n'.join('   '.join(cols[h][i] for h in header) for i in xrange(n_rows))
    table = '{header}\n{hline}\n{body}\n'.format(
        header='   '.join(header_aligned), hline='=' * table_width, body=body)
    return table

def _sleep(duration):
    rospy.rostime.wallsleep(duration)

def _rostopic_hz(topics, window_size=-1, filter_expr=None, use_wtime=False, tcp_nodelay=False):
    """
    Periodically print the publishing rate of a topic to console until
    shutdown
    :param topics: topic names, ``list`` of ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param filter_expr: Python filter expression that is called with m, the message instance
    :param tcp_nodelay: Subscribe with the TCP_NODELAY transport hint if true
    """
    _check_master()
    if rospy.is_shutdown():
        return
    rospy.init_node(NAME, anonymous=True)
    rt = ROSTopicHz(window_size, filter_expr=filter_expr, use_wtime=use_wtime)
    for topic in topics:
        msg_class, real_topic, _ = get_topic_class(topic, blocking=True) # pause hz until topic is published
        # we use a large buffer size as we don't know what sort of messages we're dealing with.
        # may parameterize this in the future
        if filter_expr is not None:
            # have to subscribe with topic_type
            rospy.Subscriber(real_topic, msg_class, rt.callback_hz, callback_args=topic, tcp_nodelay=tcp_nodelay)
        else:
            rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz, callback_args=topic, tcp_nodelay=tcp_nodelay)
        print("subscribed to [%s]" % real_topic)

    if rospy.get_param('use_sim_time', False):
        print("WARNING: may be using simulated time",file=sys.stderr)

    while not rospy.is_shutdown():
        _sleep(1.0)
        rt.print_hz(topics)

class ROSTopicDelay(object):

    def __init__(self, window_size):
        import threading
        self.lock = threading.Lock()
        self.last_msg_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.delays = []

        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

    def callback_delay(self, msg):
        if not msg._has_header:
            rospy.logerr('msg does not have header')
            return
        with self.lock:
            curr_rostime = rospy.get_rostime()

            # time reset
            if curr_rostime.is_zero():
                if len(self.delays) > 0:
                    print("time has reset, resetting counters")
                    self.delays = []
                return

            curr = curr_rostime.to_sec()
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.delays = []
            else:
                self.delays.append(curr_rostime.to_time() -
                                   msg.header.stamp.to_time())
                self.msg_tn = curr

            if len(self.delays) > self.window_size - 1:
                self.delays.pop(0)

    def get_delay(self):
        if self.msg_tn == self.last_msg_tn:
            return
        with self.lock:
            if not self.delays:
                return
            n = len(self.delays)

            mean = sum(self.delays) / n
            rate = 1. / mean if mean > 0 else 0

            std_dev = math.sqrt(sum((x - mean)**2 for x in self.delays) / n)

            max_delta = max(self.delays)
            min_delta = min(self.delays)

            self.last_msg_tn = self.msg_tn
        return mean, min_delta, max_delta, std_dev, n + 1

    def print_delay(self):
        """
        print the average publishing delay to screen
        """
        if not self.delays:
            return
        ret = self.get_delay()
        if ret is None:
            print("no new messages")
            return
        delay, min_delta, max_delta, std_dev, window = ret
        print("average delay: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(delay, min_delta, max_delta, std_dev, window))


def _rostopic_delay(topic, window_size=-1, tcp_nodelay=False):
    """
    Periodically print the publishing delay of a topic to console until
    shutdown
    :param topic: topic name, ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param tcp_nodelay: Subscribe with the TCP_NODELAY transport hint if true
    """
    # pause hz until topic is published
    msg_class, real_topic, _ = get_topic_class(topic, blocking=True)
    if rospy.is_shutdown():
        return
    rospy.init_node(NAME, anonymous=True)
    rt = ROSTopicDelay(window_size)
    sub = rospy.Subscriber(real_topic, msg_class, rt.callback_delay, tcp_nodelay=tcp_nodelay)
    print("subscribed to [%s]" % real_topic)

    if rospy.get_param('use_sim_time', False):
        print("WARNING: may be using simulated time",file=sys.stderr)

    while not rospy.is_shutdown():
        _sleep(1.0)
        rt.print_delay()


class ROSTopicBandwidth(object):
    def __init__(self, window_size=100):
        import threading
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.sizes =[]
        self.times =[]        
        self.window_size = window_size or 100
                
    def callback(self, data):
        """ros sub callback"""
        with self.lock:
            try:
                t = time.time()
                self.times.append(t)
                self.sizes.append(len(data._buff)) #AnyMsg instance
                assert(len(self.times) == len(self.sizes))

                if len(self.times) > self.window_size:
                    self.times.pop(0)
                    self.sizes.pop(0)
            except:
                traceback.print_exc()

    def print_bw(self):
        """print the average publishing rate to screen"""
        if len(self.times) < 2:
            return
        with self.lock:
            n = len(self.times)
            tn = time.time()
            t0 = self.times[0]
            
            total = sum(self.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n

            #std_dev = math.sqrt(sum((x - mean)**2 for x in self.sizes) /n)

            # min and max
            max_s = max(self.sizes)
            min_s = min(self.sizes)

        #min/max and even mean are likely to be much smaller, but for now I prefer unit consistency
        if bytes_per_s < 1000:
            bw, mean, min_s, max_s = ["%.2fB"%v for v in [bytes_per_s, mean, min_s, max_s]]
        elif bytes_per_s < 1000000:
            bw, mean, min_s, max_s = ["%.2fKB"%(v/1000) for v in [bytes_per_s, mean, min_s, max_s]]  
        else:
            bw, mean, min_s, max_s = ["%.2fMB"%(v/1000000) for v in [bytes_per_s, mean, min_s, max_s]]
            
        print("average: %s/s\n\tmean: %s min: %s max: %s window: %s"%(bw, mean, min_s, max_s, n))

def _isstring_type(t):
    valid_types = [str]
    try:
        valid_types.append(unicode)
    except NameError:
        pass
    return t in valid_types

def _rostopic_bw(topic, window_size=-1):
    """
    periodically print the received bandwidth of a topic to console until
    shutdown
    """
    _check_master()
    _, real_topic, _ = get_topic_type(topic, blocking=True) #pause hz until topic is published
    if rospy.is_shutdown():
        return
    # #3543 disable all auto-subscriptions to /clock
    rospy.init_node(NAME, anonymous=True, disable_rostime=True)
    rt = ROSTopicBandwidth(window_size)
    # we use a large buffer size as we don't know what sort of messages we're dealing with.
    # may parameterize this in the future
    sub = rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback)
    print("subscribed to [%s]"%real_topic)
    while not rospy.is_shutdown():
        _sleep(1.0)
        rt.print_bw()

# code adapted from rqt_plot
def msgevalgen(pattern):
    """
    Generates a function that returns the relevant field(s) (aka 'subtopic(s)') of a Message object
    :param pattern: subtopic, e.g. /x[2:]/y[:-1]/z, ``str``
    :returns: function that converts a message into the desired value, ``fn(Message) -> value``
    """
    evals = []  # list of (field_name, slice_object) pairs
    fields = [f for f in pattern.split('/') if f]
    for f in fields:
        if '[' in f:
            field_name, rest = f.split('[', 1)
            if not rest.endswith(']'):
                print("missing closing ']' in slice spec '%s'" % f, file=sys.stderr)
                return None
            rest = rest[:-1]  # slice content, removing closing bracket
            try:
                array_index_or_slice_object = _get_array_index_or_slice_object(rest)
            except AssertionError as e:
                print("field '%s' has invalid slice argument '%s': %s"
                      % (field_name, rest, str(e)), file=sys.stderr)
                return None
            evals.append((field_name, array_index_or_slice_object))
        else:
            evals.append((f, None))

    def msgeval(msg, evals):
        for i, (field_name, slice_object) in enumerate(evals):
            try: # access field first
                msg = getattr(msg, field_name)
            except AttributeError:
                print("no field named %s in %s" % (field_name, pattern), file=sys.stderr)
                return None

            if slice_object is not None: # access slice
                try:
                    msg = msg.__getitem__(slice_object)
                except IndexError as e:
                    print("%s: %s" % (str(e), pattern), file=sys.stderr)
                    return None

                # if a list is returned here (i.e. not only a single element accessed),
                # we need to recursively call msg_eval() with the rest of evals
                # in order to handle nested slices
                if isinstance(msg, list):
                    rest = evals[i + 1:]
                    return [msgeval(m, rest) for m in msg]
        return msg

    return (lambda msg: msgeval(msg, evals)) if evals else None


def _get_array_index_or_slice_object(index_string):
    assert index_string != '', 'empty array index'
    index_string_parts = index_string.split(':')
    if len(index_string_parts) == 1:
        try:
            array_index = int(index_string_parts[0])
        except ValueError:
            assert False, "non-integer array index step '%s'" % index_string_parts[0]
        return array_index

    slice_args = [None, None, None]
    if index_string_parts[0] != '':
        try:
            slice_args[0] = int(index_string_parts[0])
        except ValueError:
            assert False, "non-integer slice start '%s'" % index_string_parts[0]
    if index_string_parts[1] != '':
        try:
            slice_args[1] = int(index_string_parts[1])
        except ValueError:
            assert False, "non-integer slice stop '%s'" % index_string_parts[1]
    if len(index_string_parts) > 2 and index_string_parts[2] != '':
            try:
                slice_args[2] = int(index_string_parts[2])
            except ValueError:
                assert False, "non-integer slice step '%s'" % index_string_parts[2]
    if len(index_string_parts) > 3:
        assert False, 'too many slice arguments'
    return slice(*slice_args)

def _get_nested_attribute(msg, nested_attributes):
    value = msg
    for attr in nested_attributes.split('/'):
        value = getattr(value, attr)
    return value

def _get_topic_type(topic):
    """
    subroutine for getting the topic type
    :returns: topic type, real topic name and fn to evaluate the message instance
    if the topic points to a field within a topic, e.g. /rosout/msg, ``(str, str, fn)``
    """
    try:
        val = _master_get_topic_types(rosgraph.Master('/rostopic'))
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    # exact match first, followed by prefix match
    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        # choose longest match
        matches.sort(key=itemgetter(0), reverse=True)

        # try to ignore messages which don't have the field specified as part of the topic name
        while matches:
            t, t_type = matches[0]
            msg_class = roslib.message.get_message_class(t_type)
            if not msg_class:
                # if any class is not fetchable skip ignoring any message types
                break
            msg = msg_class()
            nested_attributes = topic[len(t) + 1:].rstrip('/')
            nested_attributes = nested_attributes.split('[')[0]
            if nested_attributes == '':
                break
            try:
                _get_nested_attribute(msg, nested_attributes)
            except AttributeError:
                # ignore this type since it does not have the requested field
                matches.pop(0)
                continue
            matches = [(t, t_type)]
            break

    if matches:
        t, t_type = matches[0]
        if t_type == rosgraph.names.ANYTYPE:
            return None, None, None
        return t_type, t, msgevalgen(topic[len(t):])
    else:
        return None, None, None

# NOTE: this is used externally by rxplot
    
def get_topic_type(topic, blocking=False):
    """
    Get the topic type.

    :param topic: topic name, ``str``
    :param blocking: (default False) block until topic becomes available, ``bool``
    
    :returns: topic type, real topic name and fn to evaluate the message instance
      if the topic points to a field within a topic, e.g. /rosout/msg. fn is None otherwise. ``(str, str, fn)``
    :raises: :exc:`ROSTopicException` If master cannot be contacted
    """
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    elif blocking:
        sys.stderr.write("WARNING: topic [%s] does not appear to be published yet\n"%topic)
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                _sleep(0.1)
    return None, None, None

def get_topic_class(topic, blocking=False):
    """
    Get the topic message class
    :returns: message class for topic, real topic
      name, and function for evaluating message objects into the subtopic
      (or ``None``). ``(Message, str, str)``
    :raises: :exc:`ROSTopicException` If topic type cannot be determined or loaded
    """
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % topic_type)
    return msg_class, real_topic, msg_eval

def _str_plot_fields(val, f, field_filter):
    """
    get CSV representation of fields used by _str_plot
    :returns: list of fields as a CSV string, ``str``
    """
    s = _sub_str_plot_fields(val, f, field_filter)
    if s is not None:
        return "time,"+s
    else:
        return 'time,'

def _sub_str_plot_fields(val, f, field_filter):
    """recursive helper function for _str_plot_fields"""
    # CSV
    type_ = type(val)
    if type_ in (bool, int, long, float) or \
           isinstance(val, genpy.TVal):
        return f
    # duck-type check for messages
    elif hasattr(val, "_slot_types"):
        if field_filter is not None:
            fields = list(field_filter(val))
        else:
            fields = val.__slots__
        sub = (_sub_str_plot_fields(_convert_getattr(val, a, t), f+"."+a, field_filter) for a,t in zip(val.__slots__, val._slot_types) if a in fields)
        sub = [s for s in sub if s is not None]
        if sub:
            return ','.join([s for s in sub])
    elif _isstring_type(type_):
        return f
    elif type_ in (list, tuple):
        if len(val) == 0:
            return None
        val0 = val[0]
        type0 = type(val0)
        # no arrays of arrays
        if type0 in (bool, int, long, float) or \
               isinstance(val0, genpy.TVal):
            return ','.join(["%s%s"%(f,x) for x in range(0,len(val))])
        elif _isstring_type(type0):
            
            return ','.join(["%s%s"%(f,x) for x in range(0,len(val))])
        elif hasattr(val0, "_slot_types"):
            labels = ["%s%s"%(f,x) for x in range(0,len(val))]
            sub = [s for s in [_sub_str_plot_fields(v, sf, field_filter) for v,sf in zip(val, labels)] if s]
            if sub:
                return ','.join([s for s in sub])
    return None


def _str_plot(val, time_offset=None, current_time=None, field_filter=None, type_information=None, fixed_numeric_width=None, value_transform=None):
    """
    Convert value to matlab/octave-friendly CSV string representation.

    :param val: message
    :param current_time: current :class:`genpy.Time` to use if message does not contain its own timestamp.
    :param time_offset: (optional) for time printed for message, print as offset against this :class:`genpy.Time`
    :param field_filter: filter the fields that are stringified for Messages, ``fn(Message)->iter(str)``
    :param value_transform: Not used but for same API as CallbackEcho.custom_strify_message
    :returns: comma-separated list of field values in val, ``str``
    """
        
    s = _sub_str_plot(val, time_offset, field_filter)
    if s is None:
        s = ''

    if time_offset is not None:
        time_offset = time_offset.to_nsec()
    else:
        time_offset = 0            
        
    if current_time is not None:
        return "%s,%s"%(current_time.to_nsec()-time_offset, s)
    elif getattr(val, "_has_header", False):
        return "%s,%s"%(val.header.stamp.to_nsec()-time_offset, s)
    else:
        return "%s,%s"%(rospy.get_rostime().to_nsec()-time_offset, s)
    
def _sub_str_plot(val, time_offset, field_filter):
    """Helper routine for _str_plot."""
    # CSV
    type_ = type(val)
    
    if type_ == bool:
        return '1' if val else '0'
    elif type_ in (int, long, float) or \
           isinstance(val, genpy.TVal):
        if time_offset is not None and isinstance(val, genpy.Time):
            return str(val-time_offset)
        else:
            return str(val)    
    elif hasattr(val, "_slot_types"):
        if field_filter is not None:
            fields = list(field_filter(val))
        else:
            fields = val.__slots__            

        sub = (_sub_str_plot(_convert_getattr(val, f, t), time_offset, field_filter) for f,t in zip(val.__slots__, val._slot_types) if f in fields)
        sub = [s for s in sub if s is not None]
        if sub:
            return ','.join(sub)
    elif _isstring_type(type_):
        return val
    elif type_ in (list, tuple):
        if len(val) == 0:
            return None
        val0 = val[0]
        # no arrays of arrays
        type0 = type(val0)
        if type0 == bool:
            return ','.join([('1' if v else '0') for v in val])
        elif type0 in (int, long, float) or \
               isinstance(val0, genpy.TVal):
            return ','.join([str(v) for v in val])
        elif _isstring_type(type0):
            return ','.join([v for v in val])            
        elif hasattr(val0, "_slot_types"):
            sub = [s for s in [_sub_str_plot(v, time_offset, field_filter) for v in val] if s is not None]
            if sub:
                return ','.join([s for s in sub])
    return None
        
# copied from roslib.message
def _convert_getattr(val, f, t):
    """
    Convert atttribute types on the fly, if necessary.  This is mainly
    to convert uint8[] fields back to an array type.
    """
    attr = getattr(val, f)
    if _isstring_type(type(attr)) and 'uint8[' in t:
        return [ord(x) for x in attr]
    else:
        return attr

class CallbackEcho(object):
    """
    Callback instance that can print callback data in a variety of
    formats. Used for all variants of rostopic echo
    """

    def __init__(self, topic, msg_eval, plot=False, filter_fn=None,
                 echo_clear=False, echo_all_topics=False,
                 offset_time=False, count=None,
                 field_filter_fn=None, fixed_numeric_width=None,
                 value_transform_fn=None):
        """
        :param plot: if ``True``, echo in plotting-friendly format (csv), ``bool``
        :param filter_fn: function that evaluates to ``True`` if message is to be echo'd, ``fn(topic, msg)``
        :param echo_all_topics: (optional) if ``True``, echo all messages in bag, ``bool``
        :param offset_time: (optional) if ``True``, display time as offset from current time, ``bool``
        :param count: number of messages to echo, ``None`` for infinite, ``int``
        :param field_filter_fn: filter the fields that are stringified for Messages, ``fn(Message)->iter(str)``
        :param fixed_numeric_width: fixed width for numeric values, ``None`` for automatic, ``int``
        :param value_transform_fn: transform the values of Messages, ``fn(Message)->Message``
        """
        if topic and topic[-1] == '/':
            topic = topic[:-1]
        self.topic = topic
        self.msg_eval = msg_eval
        self.plot = plot
        self.filter_fn = filter_fn
        self.fixed_numeric_width = fixed_numeric_width

        self.prefix = ''
        self.suffix = '\n---' if not plot else ''# same as YAML document separator, bug #3291
        
        self.echo_all_topics = echo_all_topics
        self.offset_time = offset_time

        # done tracks when we've exceeded the count
        self.done = False
        self.max_count = count
        self.count = 0

        # determine which strifying function to use
        if plot:
            #TODOXXX: need to pass in filter function
            self.str_fn = _str_plot
            self.sep = ''
        else:
            #TODOXXX: need to pass in filter function
            self.str_fn = self.custom_strify_message
            if echo_clear:
                self.prefix = '\033[2J\033[;H'

        self.field_filter=field_filter_fn
        self.value_transform=value_transform_fn
        
        # first tracks whether or not we've printed anything yet. Need this for printing plot fields.
        self.first = True

        # cache
        self.last_topic = None
        self.last_msg_eval = None

    def custom_strify_message(self, val, indent='', time_offset=None, current_time=None, field_filter=None,
                              type_information=None, fixed_numeric_width=None, value_transform=None):
        # ensure to print uint8[] as array of numbers instead of string
        if type_information and type_information.startswith('uint8['):
            val = [ord(x) for x in val]
        if value_transform is not None:
            val = value_transform(val, type_information)
        return genpy.message.strify_message(val, indent=indent, time_offset=time_offset, current_time=current_time, field_filter=field_filter, fixed_numeric_width=fixed_numeric_width)

    def callback(self, data, callback_args, current_time=None):
        """
        Callback to pass to rospy.Subscriber or to call
        manually. rospy.Subscriber constructor must also pass in the
        topic name as an additional arg
        :param data: Message
        :param topic: topic name, ``str``
        :param current_time: override calculation of current time, :class:`genpy.Time`
        """
        topic = callback_args['topic']
        type_information = callback_args.get('type_information', None)
        if self.filter_fn is not None and not self.filter_fn(data):
            return

        if self.max_count is not None and self.count >= self.max_count:
            self.done = True
            return
        
        try:
            msg_eval = self.msg_eval
            if topic == self.topic:
                pass
            elif self.topic.startswith(topic + '/'):
                # self.topic is actually a reference to topic field, generate msgeval
                if topic == self.last_topic:
                    # use cached eval
                    msg_eval = self.last_msg_eval
                else:
                    # generate msg_eval and cache
                    self.last_msg_eval = msg_eval = msgevalgen(self.topic[len(topic):])
                    self.last_topic = topic
            elif not self.echo_all_topics:
                return

            if msg_eval is not None:
                data = msg_eval(data)
                
            # data can be None if msg_eval returns None
            if data is not None:
                # NOTE: we do all prints using direct writes to sys.stdout, which works better with piping
                
                self.count += 1
                
                # print fields header for plot
                if self.plot and self.first:
                    sys.stdout.write("%"+_str_plot_fields(data, 'field', self.field_filter)+'\n')
                    self.first = False

                if self.offset_time:
                    sys.stdout.write(self.prefix+\
                                     self.str_fn(data, time_offset=rospy.get_rostime(),
                                                 current_time=current_time, field_filter=self.field_filter,
                                                 type_information=type_information, fixed_numeric_width=self.fixed_numeric_width,
                                                 value_transform=self.value_transform) + \
                                     self.suffix + '\n')
                else:
                    sys.stdout.write(self.prefix+\
                                     self.str_fn(data,
                                                 current_time=current_time, field_filter=self.field_filter,
                                                 type_information=type_information, fixed_numeric_width=self.fixed_numeric_width,
                                                 value_transform=self.value_transform) + \
                                     self.suffix + '\n')

                # we have to flush in order before piping to work
                sys.stdout.flush()
            # #2778 : have to check count after incr to set done flag
            if self.max_count is not None and self.count >= self.max_count:
                self.done = True

        except IOError:
            self.done = True
        except:
            # set done flag so we exit
            self.done = True
            traceback.print_exc()
            
def _rostopic_type(topic):
    """
    Print ROS message type of topic to screen
    :param topic: topic name, ``str``
    """
    topic_type, topic_real_name, _ = get_topic_type(topic, blocking=False)
    if topic_type is None:
        sys.stderr.write('unknown topic type [%s]\n'%topic)
        sys.exit(1)
    elif topic == topic_real_name:
        print(topic_type)
    else:
        field = topic[len(topic_real_name)+1:]
        field_type = topic_type
        for current_field in field.split('/'):
            msg_class = roslib.message.get_message_class(field_type)
            field_type = msg_class._slot_types[msg_class.__slots__.index(current_field)]
        print('%s %s %s'%(topic_type, field, field_type))

def _rostopic_echo_bag(callback_echo, bag_file):
    """
    :param callback_echo: :class:`CallbackEcho` instance to invoke on new messages in bag file
    :param bag_file: name of bag file to echo messages from or ``None``, ``str``
    """
    if not os.path.exists(bag_file):
        raise ROSTopicException("bag file [%s] does not exist"%bag_file)
    first = True
    
    import rosbag
    with rosbag.Bag(bag_file) as b:
        for t, msg, timestamp in b.read_messages():
        # bag files can have relative paths in them, this respects any
            # dynamic renaming
            if t[0] != '/':
                t = rosgraph.names.script_resolve_name('rostopic', t)
            callback_echo.callback(msg, {'topic': t}, current_time=timestamp)
            # done is set if there is a max echo count
            if callback_echo.done:
                break

def _rostopic_echo(topic, callback_echo, bag_file=None, echo_all_topics=False):
    """
    Print new messages on topic to screen.
    
    :param topic: topic name, ``str``
    :param bag_file: name of bag file to echo messages from or ``None``, ``str``
    """
    # we have to init a node regardless and bag echoing can print timestamps

    if bag_file:
        # initialize rospy time due to potential timestamp printing
        rospy.rostime.set_rostime_initialized(True)        
        _rostopic_echo_bag(callback_echo, bag_file)
    else:
        _check_master()
        rospy.init_node(NAME, anonymous=True)
        msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
        if msg_class is None:
            # occurs on ctrl-C
            return
        callback_echo.msg_eval = msg_eval

        # extract type information for submessages
        type_information = None
        if len(topic) > len(real_topic):
            subtopic = topic[len(real_topic):]
            subtopic = subtopic.strip('/')
            if subtopic:
                fields = subtopic.split('/')
                submsg_class = msg_class
                while fields:
                    field = fields[0].split('[')[0]
                    del fields[0]
                    index = submsg_class.__slots__.index(field)
                    type_information = submsg_class._slot_types[index]
                    if fields:
                        submsg_class = roslib.message.get_message_class(type_information.split('[', 1)[0])
                        if not submsg_class:
                            raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % type_information)

        use_sim_time = rospy.get_param('/use_sim_time', False)
        sub = rospy.Subscriber(real_topic, msg_class, callback_echo.callback, {'topic': topic, 'type_information': type_information})

        if use_sim_time:
            # #2950: print warning if nothing received for two seconds

            timeout_t = time.time() + 2.
            while time.time() < timeout_t and \
                    callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                _sleep(0.1)

            if callback_echo.count == 0 and \
                    not rospy.is_shutdown() and \
                    not callback_echo.done:
                sys.stderr.write("WARNING: no messages received and simulated time is active.\nIs /clock being published?\n")

        while not rospy.is_shutdown() and not callback_echo.done:
            _sleep(0.1)

_caller_apis = {}
def get_api(master, caller_id):
    """
    Get XML-RPC API of node
    :param master: XML-RPC handle to ROS Master, :class:`xmlrpclib.ServerProxy`
    :param caller_id: node name, ``str``
    :returns: XML-RPC URI of node, ``str``
    :raises: :exc:`ROSTopicIOException` If unable to communicate with master
    """
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        except rosgraph.MasterError:
            caller_api = 'unknown address %s'%caller_id

    return caller_api

def _rostopic_list_bag(bag_file, topic=None):
    """
    Prints topics in bag file to screen
    :param bag_file: path to bag file, ``str``
    :param topic: optional topic name to match. Will print additional information just about messagese in this topic, ``str``
    """
    import rosbag
    if not os.path.exists(bag_file):
        raise ROSTopicException("bag file [%s] does not exist"%bag_file)

    with rosbag.Bag(bag_file) as b:
        if topic:
            # create string for namespace comparison
            topic_ns = rosgraph.names.make_global_ns(topic)
            count = 0
            earliest = None
            latest = None
            for top, msg, t in b.read_messages(raw=True):
                if top == topic or top.startswith(topic_ns):
                    count += 1
                    if earliest == None:
                        earliest = t

                    latest = t
                if rospy.is_shutdown():
                    break
            import time
            earliest, latest = [time.strftime("%d %b %Y %H:%M:%S", time.localtime(t.to_time())) for t in (earliest, latest)]
            print("%s message(s) from %s to %s"%(count, earliest, latest))
        else:
            topics = set()
            for top, msg, _ in b.read_messages(raw=True):
                if top not in topics:
                    print(top)
                    topics.add(top)
                if rospy.is_shutdown():
                    break

def _sub_rostopic_list(master, pubs, subs, publishers_only, subscribers_only, verbose, indent=''):
    if verbose:
        topic_types = _master_get_topic_types(master)

        if not subscribers_only:
            print("\n%sPublished topics:"%indent)
            for t, ttype, tlist in pubs:
                if len(tlist) > 1:
                    print(indent+" * %s [%s] %s publishers"%(t, ttype, len(tlist)))
                else:
                    print(indent+" * %s [%s] 1 publisher"%(t, ttype))                    

        if not publishers_only:
            print(indent)
            print(indent+"Subscribed topics:")
            for t, ttype, tlist in subs:
                if len(tlist) > 1:
                    print(indent+" * %s [%s] %s subscribers"%(t, ttype, len(tlist)))
                else:
                    print(indent+" * %s [%s] 1 subscriber"%(t, ttype))
        print('')
    else:
        if publishers_only:
            topics = [t for t, _, _ in pubs]
        elif subscribers_only:
            topics = [t for t, _, _ in subs]
        else:
            topics = list(set([t for t, _, _ in pubs] + [t for t, _, _ in subs]))                
        topics.sort()
        print('\n'.join(["%s%s"%(indent, t) for t in topics]))

def get_topic_list(master=None):
    if not master:
        master = rosgraph.Master('/rostopic')
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    # Return an array of tuples; (<topic>, <type>, <node_count>)
    state = master.getSystemState()
    topic_types = _master_get_topic_types(master)
    
    pubs, subs, _ = state
    pubs_out = []
    for topic, nodes in pubs:
        pubs_out.append((topic, topic_type(topic, topic_types), nodes))
    subs_out = []
    for topic, nodes in subs:
        subs_out.append((topic, topic_type(topic, topic_types), nodes))

    # List of published topics, list of subscribed topics.
    return (pubs_out, subs_out)

# #3145
def _rostopic_list_group_by_host(master, pubs, subs):
    """
    Build up maps for hostname to topic list per hostname
    :returns: publishers host map, subscribers host map, ``{str: set(str)}, {str: set(str)}``
    """
    def build_map(master, state, uricache):
        tmap = {}
        for topic, ttype, tnodes in state:
            for p in tnodes:
                if not p in uricache:
                   uricache[p] = master.lookupNode(p)
                uri = uricache[p]
                puri = urlparse(uri)
                if not puri.hostname in tmap:
                    tmap[puri.hostname] = []
                # recreate the system state data structure, but for a single host
                matches = [l for x, _, l in tmap[puri.hostname] if x == topic]
                if matches:
                    matches[0].append(p)
                else:
                    tmap[puri.hostname].append((topic, ttype, [p]))
        return tmap
        
    uricache = {}
    host_pub_topics = build_map(master, pubs, uricache)
    host_sub_topics = build_map(master, subs, uricache)
    return host_pub_topics, host_sub_topics

def _rostopic_list(topic, verbose=False,
                   subscribers_only=False, publishers_only=False,
                   group_by_host=False):
    """
    Print topics to screen
    
    :param topic: topic name to list information or None to match all topics, ``str``
    :param verbose: print additional debugging information, ``bool``
    :param subscribers_only: print information about subscriptions only, ``bool``
    :param publishers_only: print information about subscriptions only, ``bool``
    :param group_by_host: group topic list by hostname, ``bool``
    """
    # #1563
    if subscribers_only and publishers_only:
        raise ROSTopicException("cannot specify both subscribers- and publishers-only")
    
    master = rosgraph.Master('/rostopic')
    try:
        pubs, subs = get_topic_list(master=master)
        if topic:
            topic_ns = rosgraph.names.make_global_ns(topic)
            subs = (x for x in subs if x[0] == topic or x[0].startswith(topic_ns))
            pubs = (x for x in pubs if x[0] == topic or x[0].startswith(topic_ns))
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    if group_by_host:
        # #3145
        host_pub_topics, host_sub_topics  = _rostopic_list_group_by_host(master, pubs, subs)
        for hostname in set(list(host_pub_topics.keys()) + list(host_sub_topics.keys())): #py3k
            pubs, subs = host_pub_topics.get(hostname,[]), host_sub_topics.get(hostname, []),
            if (pubs and not subscribers_only) or (subs and not publishers_only):
                print("Host [%s]:" % hostname)
                _sub_rostopic_list(master, pubs, subs,
                                   publishers_only, subscribers_only,
                                   verbose, indent='  ')
    else:
        _sub_rostopic_list(master, pubs, subs,
                           publishers_only, subscribers_only,
                           verbose)

def get_info_text(topic):
    """
    Get human-readable topic description
    
    :param topic: topic name, ``str``
    """
    try:
        from cStringIO import StringIO
    except ImportError:
        from io import StringIO
    import itertools
    buff = StringIO()
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master('/rostopic')
    try:
        pubs, subs = get_topic_list(master=master)
        # filter based on topic
        subs = [x for x in subs if x[0] == topic]
        pubs = [x for x in pubs if x[0] == topic]

        topic_types = _master_get_topic_types(master)
            
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    if not pubs and not subs:
        raise ROSTopicException("Unknown topic %s"%topic)

    buff.write("Type: %s\n\n"%topic_type(topic, topic_types))

    if pubs:
        buff.write("Publishers: \n")
        for p in itertools.chain(*[nodes for topic, ttype, nodes in pubs]):
            buff.write(" * %s (%s)\n"%(p, get_api(master, p)))
    else:
        buff.write("Publishers: None\n")
    buff.write('\n')

    if subs:
        buff.write("Subscribers: \n")
        for p in itertools.chain(*[nodes for topic, ttype, nodes in subs]):
            buff.write(" * %s (%s)\n"%(p, get_api(master, p)))
    else:
        buff.write("Subscribers: None\n")
    buff.write('\n')
    return buff.getvalue()
    
def _rostopic_info(topic):
    """
    Print topic information to screen.
    
    :param topic: topic name, ``str``
    """
    print(get_info_text(topic))
            
##########################################################################################
# COMMAND PROCESSING #####################################################################
    
def _rostopic_cmd_echo(argv):
    def expr_eval(expr):
        def eval_fn(m):
            return eval(expr)
        return eval_fn

    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog echo [options] /topic", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="echo messages from .bag file", metavar="BAGFILE")
    parser.add_option("-p", 
                      dest="plot", default=False,
                      action="store_true",
                      help="echo in a plotting friendly format")
    parser.add_option("-w",
                      dest="fixed_numeric_width", default=None, metavar="NUM_WIDTH",
                      help="fixed width for numeric values")
    parser.add_option("--filter", 
                      dest="filter_expr", default=None,
                      metavar="FILTER-EXPRESSION",
                      help="Python expression to filter messages that are printed. Expression can use Python builtins as well as m (the message) and topic (the topic name).")
    parser.add_option("--nostr", 
                      dest="nostr", default=False,
                      action="store_true",
                      help="exclude string fields")
    parser.add_option("--noarr",
                      dest="noarr", default=False,
                      action="store_true",
                      help="exclude arrays")
    parser.add_option("-c", "--clear",
                      dest="clear", default=False,
                      action="store_true",
                      help="clear screen before printing next message")
    parser.add_option("-a", "--all",
                      dest="all_topics", default=False,
                      action="store_true",
                      help="display all message in bag, only valid with -b option")
    parser.add_option("-n", 
                      dest="msg_count", default=None, metavar="COUNT",
                      help="number of messages to echo")
    parser.add_option("--offset",
                      dest="offset_time", default=False,
                      action="store_true",
                      help="display time as offsets from current time (in seconds)")

    (options, args) = parser.parse_args(args)
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    if options.all_topics and not options.bag:
        parser.error("Display all option is only valid when echoing from bag files")
    if options.offset_time and options.bag:
        parser.error("offset time option is not valid with bag files")
    if options.all_topics:
        topic = ''
    else:
        if len(args) == 0:
            parser.error("topic must be specified")        
        topic = rosgraph.names.script_resolve_name('rostopic', args[0])
        # suppressing output to keep it clean
        #if not options.plot:
        #    print "rostopic: topic is [%s]"%topic
        
    filter_fn = None
    if options.filter_expr:
        filter_fn = expr_eval(options.filter_expr)

    try:
        msg_count = int(options.msg_count) if options.msg_count else None
    except ValueError:
        parser.error("COUNT must be an integer")

    try:
        fixed_numeric_width = int(options.fixed_numeric_width) if options.fixed_numeric_width else None
        if fixed_numeric_width is not None and fixed_numeric_width < 2:
            parser.error("Fixed width for numeric values must be at least 2")
    except ValueError:
        parser.error("NUM_WIDTH must be an integer")

    if options.plot:
        field_filter_fn = create_field_filter(options.nostr, options.noarr)
        value_transform_fn = None
    else:
        field_filter_fn = None
        value_transform_fn = create_value_transform(options.nostr, options.noarr)

    callback_echo = CallbackEcho(topic, None, plot=options.plot,
                                 filter_fn=filter_fn,
                                 echo_clear=options.clear, echo_all_topics=options.all_topics,
                                 offset_time=options.offset_time, count=msg_count,
                                 field_filter_fn=field_filter_fn,
                                 value_transform_fn=value_transform_fn,
                                 fixed_numeric_width=fixed_numeric_width)
    try:
        _rostopic_echo(topic, callback_echo, bag_file=options.bag)
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")

def create_value_transform(echo_nostr, echo_noarr):
    def value_transform(val, type_information=None):
        def transform_field_value(value, value_type, echo_nostr, echo_noarr):
            if echo_noarr and '[' in value_type:
                return '<array type: %s, length: %s>' % \
                    (value_type.rstrip('[]'), len(value))
            elif echo_nostr and value_type == 'string':
                return '<string length: %s>' % len(value)
            elif echo_nostr and value_type == 'string[]':
                return '<array type: string, length: %s>' % len(value)
            return value

        if not isinstance(val, genpy.Message):
            if type_information is None:
                return val
            return transform_field_value(val, type_information,
                                         echo_nostr, echo_noarr)

        class TransformedMessage(genpy.Message):
            # These should be copy because changing these variables
            # in transforming is problematic without its untransforming.
            __slots__ = val.__slots__[:]
            _slot_types = val._slot_types[:]

        val_trans = TransformedMessage()

        fields = val.__slots__
        field_types = val._slot_types
        for index, (f, t) in enumerate(zip(fields, field_types)):
            f_val = getattr(val, f)
            f_val_trans = transform_field_value(f_val, t,
                                                echo_nostr, echo_noarr)
            if f_val_trans != f_val:
                setattr(val_trans, f, f_val_trans)
                val_trans._slot_types[index] = 'string'
            else:
                try:
                    msg_class = genpy.message.get_message_class(t)
                    if msg_class is None:
                        # happens for list of ROS messages like std_msgs/String[]
                        raise ValueError
                    nested_transformed = value_transform(f_val)
                    setattr(val_trans, f, nested_transformed)
                except ValueError:
                    setattr(val_trans, f, f_val)
        return val_trans
    return value_transform

def create_field_filter(echo_nostr, echo_noarr):
    def field_filter(val):
        fields = val.__slots__
        field_types = val._slot_types
        for f, t in zip(val.__slots__, val._slot_types):
            if echo_noarr and '[' in t:
                continue
            elif echo_nostr and 'string' in t:
                continue
            yield f
    return field_filter

def _optparse_topic_only(cmd, argv):
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %%prog %s /topic"%cmd, prog=NAME)
    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")        
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    return rosgraph.names.script_resolve_name('rostopic', args[0])

def _rostopic_cmd_type(argv):
    parser = argparse.ArgumentParser(prog='%s type' % NAME)
    parser.add_argument('topic_or_field', help='Topic or field name')
    args = parser.parse_args(argv[2:])
    _rostopic_type(rosgraph.names.script_resolve_name('rostopic', args.topic_or_field))

def _rostopic_cmd_hz(argv):
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog hz [options] /topic_0 [/topic_1 [topic_2 [..]]]", prog=NAME)
    parser.add_option("-w", "--window",
                      dest="window_size", default=-1,
                      help="window size, in # of messages, for calculating rate", metavar="WINDOW")
    parser.add_option("--filter",
                      dest="filter_expr", default=None,
                      help="only measure messages matching the specified Python expression", metavar="EXPR")
    parser.add_option("--wall-time",
                      dest="use_wtime", default=False, action="store_true",
                      help="calculates rate using wall time which can be helpful when clock isn't published during simulation")
    parser.add_option("--tcpnodelay",
                      dest="tcp_nodelay", action="store_true",
                      help="use the TCP_NODELAY transport hint when subscribing to topics")

    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")        
    try:
        window_size = int(options.window_size)
    except:
        parser.error("window size must be an integer")

    topics = [rosgraph.names.script_resolve_name('rostopic', t) for t in args]

    # #694
    if options.filter_expr:
        def expr_eval(expr):
            def eval_fn(m):
                return eval(expr)
            return eval_fn
        filter_expr = expr_eval(options.filter_expr)
    else:
        filter_expr = None
    _rostopic_hz(topics, window_size=window_size, filter_expr=filter_expr,
                 use_wtime=options.use_wtime, tcp_nodelay=options.tcp_nodelay)


def _rostopic_cmd_delay(argv):
    args = argv[2:]
    import argparse
    parser = argparse.ArgumentParser(usage="%(prog)s delay [options] /topic", prog=NAME)
    parser.add_argument("topic", help="topic name to be calcurated the delay")
    parser.add_argument("-w", "--window",
                        dest="window_size", default=-1, type=int,
                        help="window size, in # of messages, for calculating rate")
    parser.add_argument("--tcpnodelay",
                        dest="tcp_nodelay", action="store_true",
                        help="use the TCP_NODELAY transport hint when subscribing to topics")

    args = parser.parse_args(args)
    topic_name = args.topic
    window_size = args.window_size
    topic = rosgraph.names.script_resolve_name('rostopic', topic_name)
    _rostopic_delay(topic, window_size=window_size, tcp_nodelay=args.tcp_nodelay)


def _rostopic_cmd_bw(argv=sys.argv):
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog bw /topic", prog=NAME)
    parser.add_option("-w", "--window",
                      dest="window_size", default=None,
                      help="window size, in # of messages, for calculating rate", metavar="WINDOW")
    options, args = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")        
    if len(args) > 1:
        parser.error("you may only specify one input topic")
    try:
        window_size = int(options.window_size) if options.window_size is not None else None
    except:
        parser.error("window size must be an integer")
    topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    _rostopic_bw(topic, window_size=window_size)

def find_by_type(topic_type):
    """
    Lookup topics by topic_type
    :param topic_type: type of topic to find, ``str``
    :returns: list of topic names that use topic_type, ``[str]``   
    """
    master = rosgraph.Master('/rostopic')
    try:
        t_list = _master_get_topic_types(master)
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")
    return [t_name for t_name, t_type in t_list if t_type == topic_type]
    
def _rostopic_cmd_find(argv=sys.argv):
    """
    Implements 'rostopic type'
    :param argv: command-line args, ``[str]``
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog find msg-type", prog=NAME)
    options, args = parser.parse_args(args)
    if not len(args):
        parser.error("please specify a message type")
    if len(args) > 1:
        parser.error("you may only specify one message type")
    print('\n'.join(find_by_type(args[0])))
    

def _resource_name_package(name):
    """
    pkg/typeName -> pkg, typeName -> None
    
    :param name: package resource name, e.g. 'std_msgs/String', ``str``
    :returns: package name of resource, ``str``
    """    
    if not '/' in name:
        return None
    return name[:name.find('/')]

def create_publisher(topic_name, topic_type, latch, disable_rostime=True):
    """
    Create rospy.Publisher instance from the string topic name and
    type. This is a powerful method as it allows creation of
    rospy.Publisher and Message instances using the topic and type
    names. This enables more dynamic publishing from Python programs.

    :param topic_name: name of topic, ``str``
    :param topic_type: name of topic type, ``str``
    :param latch: latching topic, ``bool``
    :param latch: disable_rostime: whether to disable rostime (use walltime instead), ``bool``
    :returns: topic :class:`rospy.Publisher`, :class:`Message` class
    """
    topic_name = rosgraph.names.script_resolve_name('rostopic', topic_name)
    try:
        msg_class = roslib.message.get_message_class(topic_type)
    except:
        raise ROSTopicException("invalid topic type: %s"%topic_type)
    if msg_class is None:
        pkg = _resource_name_package(topic_type)
        raise ROSTopicException("invalid message type: %s.\nIf this is a valid message type, perhaps you need to type 'rosmake %s'"%(topic_type, pkg))
    # disable /rosout and /rostime as this causes blips in the pubsub network due to rostopic pub often exiting quickly
    rospy.init_node('rostopic', anonymous=True, disable_rosout=True, disable_rostime=disable_rostime)
    pub = rospy.Publisher(topic_name, msg_class, latch=latch, queue_size=100)
    return pub, msg_class

def _publish_at_rate(pub, msg, rate, verbose=False, substitute_keywords=False, pub_args=None):
    """
    Publish message at specified rate. Subroutine of L{publish_message()}.
    
    :param pub: :class:rospy.Publisher` instance for topic
    :param msg: message instance to publish
    :param rate: publishing rate (hz) or None for just once, ``int``
    :param verbose: If ``True``, print more verbose output to stdout, ``bool``
    """
    try:
        r = rospy.Rate(float(rate))
    except ValueError:
        raise ROSTopicException("Rate must be a number")
    while not rospy.is_shutdown():
        if substitute_keywords:
            _fillMessageArgs(msg, pub_args)
        if verbose:
            print("publishing %s"%msg)
        pub.publish(msg)
        r.sleep()

_ONCE_DELAY = 3.
def _publish_latched(pub, msg, once=False, verbose=False):
    """
    Publish and latch message. Subroutine of L{publish_message()}.
    
    :param pub: :class:`rospy.Publisher` instance for topic
    :param msg: message instance to publish
    :param once: if ``True``, publish message once and then exit after sleep interval, ``bool``
    :param verbose: If ``True``, print more verbose output to stdout, ``bool``
    """
    try:
        pub.publish(msg)
    except TypeError as e:
        raise ROSTopicException(str(e))

    if not once:
        rospy.spin()        

def publish_message(pub, msg_class, pub_args, rate=None, once=False, verbose=False, substitute_keywords=False):
    """
    Create new instance of msg_class, populate with pub_args, and publish. This may
    print output to screen.
    
    :param pub: :class:`rospy.Publisher` instance for topic
    :param msg_class: Message type, ``Class``
    :param pub_args: Arguments to initialize message that is published, ``[val]``
    :param rate: publishing rate (hz) or None for just once, ``int``
    :param once: publish only once and return, ``bool``
    :param verbose: If ``True``, print more verbose output to stdout, ``bool``
    """
    msg = msg_class()

    _fillMessageArgs(msg, pub_args)

    try:
        
        if rate is None:
            s = "publishing and latching [%s]"%(msg) if verbose else "publishing and latching message"
            if once:
                s = s + " for %s seconds"%_ONCE_DELAY
            else:
                s = s + ". Press ctrl-C to terminate"
            print(s)

            _publish_latched(pub, msg, once, verbose)
        else:
            _publish_at_rate(pub, msg, rate, verbose=verbose, substitute_keywords=substitute_keywords, pub_args=pub_args)
            
    except rospy.ROSSerializationException as e:
        import rosmsg
        # we could just print the message definition, but rosmsg is more readable
        raise ROSTopicException("Unable to publish message. One of the fields has an incorrect type:\n"+\
                                "  %s\n\nmsg file:\n%s"%(e, rosmsg.get_msg_text(msg_class._type)))

def _fillMessageArgs(msg, pub_args):
    try:
        # Populate the message and enable substitution keys for 'now'
        # and 'auto'. There is a corner case here: this logic doesn't
        # work if you're publishing a Header only and wish to use
        # 'auto' with it. This isn't a troubling case, but if we start
        # allowing more keys in the future, it could become an actual
        # use case. It greatly complicates logic because we'll have to
        # do more reasoning over types. to avoid ambiguous cases
        # (e.g. a std_msgs/String type, which only has a single string
        # field).

        # allow the use of the 'now' string with timestamps and 'auto' with header
        now = rospy.get_rostime()
        import std_msgs.msg
        keys = { 'now': now, 'auto': std_msgs.msg.Header(stamp=now) }
        genpy.message.fill_message_args(msg, pub_args, keys=keys)
    except genpy.MessageException as e:
        raise ROSTopicException(str(e)+"\n\nArgs are: [%s]"%genpy.message.get_printable_message_args(msg))
    
def _rostopic_cmd_pub(argv):
    """
    Parse 'pub' command arguments and run command. Will cause a system
    exit if command-line argument parsing fails.
    :param argv: command-line arguments
    :param argv: [str]
    :raises: :exc:`ROSTopicException` If call command cannot be executed
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog pub /topic type [args...]", prog=NAME)
    parser.add_option("-v", dest="verbose", default=False,
                      action="store_true",
                      help="print verbose output")
    parser.add_option("-r", "--rate", dest="rate", default=None,
                      help="publishing rate (hz).  For -f and stdin input, this defaults to 10.  Otherwise it is not set.")
    parser.add_option("-1", "--once", action="store_true", dest="once", default=False,
                      help="publish one message and exit")
    parser.add_option("-f", '--file', dest="file", metavar='FILE', default=None,
                      help="read args from YAML file (Bagy)")
    parser.add_option("-l", '--latch', dest="latch", default=False, action="store_true",
                      help="enable latching for -f, -r and piped input.  This latches the first message.")
    parser.add_option("-s", '--substitute-keywords', dest="substitute_keywords", default=False, action="store_true",
                      help="When publishing with a rate, performs keyword ('now' or 'auto') substitution for each message")
    parser.add_option('--use-rostime', dest="use_rostime", default=False, action="store_true",
                      help="use rostime for time stamps, else walltime is used")
    #parser.add_option("-p", '--param', dest="parameter", metavar='/PARAM', default=None,
    #                  help="read args from ROS parameter (Bagy format)")
    
    (options, args) = parser.parse_args(args)
    if options.rate is not None:
        if options.once:
            parser.error("You cannot select both -r and -1 (--once)")
        try:
            rate = float(options.rate)
        except ValueError:
            parser.error("rate must be a number")
        if rate <= 0:
            parser.error("rate must be greater than zero")
    else:
        # we will default this to 10 for file/stdin later
        rate = None
        
    # validate args len
    if len(args) == 0:
        parser.error("/topic must be specified")
    if len(args) == 1:
        parser.error("topic type must be specified")
    if 0:
        if len(args) > 2 and options.parameter:
            parser.error("args conflict with -p setting")        
    if len(args) > 2 and options.file:
        parser.error("args conflict with -f setting")        
    topic_name, topic_type = args[0], args[1]

    # type-case using YAML
    try:
        pub_args = []
        for arg in args[2:]:
            pub_args.append(yaml.safe_load(arg))
    except Exception as e:
        parser.error("Argument error: "+str(e))

    # make sure master is online. we wait until after we've parsed the
    # args to do this so that syntax errors are reported first
    _check_master()

    # if no rate, or explicit latch, we latch
    latch = (rate == None) or options.latch
    pub, msg_class = create_publisher(topic_name, topic_type, latch, disable_rostime=not options.use_rostime)

    if 0 and options.parameter:
        param_name = rosgraph.names.script_resolve_name('rostopic', options.parameter)
        if options.once:
            param_publish_once(pub, msg_class, param_name, rate, options.verbose)
        else:
            param_publish(pub, msg_class, param_name, rate, options.verbose)
        
    elif not pub_args and len(msg_class.__slots__):
        if not options.file and sys.stdin.isatty():
            parser.error("Please specify message values")
        # stdin/file input has a rate by default
        if rate is None and not options.latch and not options.once:
            rate = 10.
        stdin_publish(pub, msg_class, rate, options.once, options.file, options.verbose, substitute_keywords=options.substitute_keywords)
    else:
        argv_publish(pub, msg_class, pub_args, rate, options.once, options.verbose, substitute_keywords=options.substitute_keywords)
        

def file_yaml_arg(filename):
    """
    :param filename: file name, ``str``
    :returns: Iterator that yields pub args (list of args), ``iterator``
    :raises: :exc:`ROSTopicException` If filename is invalid
    """
    if not os.path.isfile(filename):
        raise ROSTopicException("file does not exist: %s"%(filename))
    import yaml
    def bagy_iter():
        try:
            with open(filename, 'r') as f:
                # load all documents
                data = yaml.safe_load_all(f)
                for d in data:
                    yield [d]
        except yaml.YAMLError as e:
            raise ROSTopicException("invalid YAML in file: %s"%(str(e)))
    return bagy_iter
    
def argv_publish(pub, msg_class, pub_args, rate, once, verbose, substitute_keywords=False):
    publish_message(pub, msg_class, pub_args, rate, once, verbose=verbose, substitute_keywords=substitute_keywords)

    if once:
        # stick around long enough for others to grab
        timeout_t = time.time() + _ONCE_DELAY
        while not rospy.is_shutdown() and time.time() < timeout_t:
            rospy.sleep(0.2)

SUBSCRIBER_TIMEOUT = 5.
def wait_for_subscriber(pub, timeout):
    timeout_t = time.time() + timeout
    while pub.get_num_connections() == 0 and timeout_t > time.time():
        _sleep(0.01)

def param_publish_once(pub, msg_class, param_name, verbose):
    if not rospy.has_param(param_name):
        raise ROSTopicException("parameter does not exist: %s"%(param_name))
    pub_args = rospy.get_param(param_name)
    argv_publish(pub, msg_class, pub_args, None, True, verbose)    


class _ParamNotifier(object):

    def __init__(self, param_name, value=None):
        import threading
        self.lock = threading.Condition()
        self.param_name = param_name
        self.updates = []
        self.value = None

    def __call__(self, key, value):
        with self.lock:
            # have to address downward if we got notification on sub namespace
            if key != self.param_name:
                subs = [x for x in key[len(self.param_name):].split('/') if x]
                idx = self.value
                for s in subs[:-1]:
                    if s in idx:
                        idx = idx[s]
                    else:
                        idx[s] = {}
                        idx = idx[s]
                idx[subs[-1]] = value
            else:
                self.value = value

            self.updates.append(self.value)
            self.lock.notify_all()
        
def param_publish(pub, msg_class, param_name, rate, verbose):
    """
    :param param_name: ROS parameter name, ``str``
    :returns: List of msg dicts in file, ``[{str: any}]``
    :raises: :exc:`ROSTopicException` If parameter is not set
    """
    import rospy
    import rospy.impl.paramserver
    import rosgraph
    
    if not rospy.has_param(param_name):
        raise ROSTopicException("parameter does not exist: %s"%(param_name))

    # reach deep into subscription APIs here. Very unstable stuff
    # here, don't copy elsewhere!
    ps_cache = rospy.impl.paramserver.get_param_server_cache()
    notifier = _ParamNotifier(param_name)
    ps_cache.set_notifier(notifier)
    master = rosgraph.Master(rospy.get_name())
    notifier.value = master.subscribeParam(rospy.get_node_uri(), param_name)
    pub_args = notifier.value
    ps_cache.set(param_name, pub_args)
    if type(pub_args) == dict:
        pub_args = [pub_args]
    elif type(pub_args) != list:
        raise ROSTopicException("Parameter [%s] in not a valid type"%(param_name))

    r = rospy.Rate(rate) if rate is not None else None
    publish = True
    while not rospy.is_shutdown():
        try:
            if publish:
                publish_message(pub, msg_class, pub_args, None, True, verbose=verbose)
        except ValueError as e:
            sys.stderr.write("%s\n"%str(e))
            break
        if r is not None:
            r.sleep()
            with notifier.lock:
                if notifier.updates:
                    pub_args = notifier.updates.pop(0)
                    if type(pub_args) == dict:
                        pub_args = [pub_args]
        else:
            publish = False
            with notifier.lock:
                if not notifier.updates:
                    notifier.lock.wait(1.)
                if notifier.updates:
                    publish = True
                    pub_args = notifier.updates.pop(0)
                    if type(pub_args) == dict:
                        pub_args = [pub_args]
            
        if rospy.is_shutdown():
            break

def stdin_publish(pub, msg_class, rate, once, filename, verbose, substitute_keywords=False):
    """
    :param filename: name of file to read from instead of stdin, or ``None``, ``str``
    """
    exactly_one_message = False

    if filename:
        iterator = file_yaml_arg(filename)

        for _ in iterator():
            if exactly_one_message:
                exactly_one_message = False
                break
            exactly_one_message = True
    else:
        iterator = stdin_yaml_arg

    r = rospy.Rate(rate) if rate is not None else None

    # stdin publishing can happen really fast, especially if no rate
    # is set, so try to make sure someone is listening before we
    # publish, though we don't wait too long.
    wait_for_subscriber(pub, SUBSCRIBER_TIMEOUT)

    for pub_args in iterator():
        if rospy.is_shutdown():
            break
        if pub_args:
            if type(pub_args) != list:
                pub_args = [pub_args]
            try:
                # if exactly one message is provided and rate is not
                # None, repeatedly publish it
                if exactly_one_message and rate is not None:
                    print("Got one message and a rate, publishing repeatedly")
                    publish_message(pub, msg_class, pub_args, rate=rate, once=once, verbose=verbose, substitute_keywords=substitute_keywords)
                # we use 'bool(r) or once' for the once value, which
                # controls whether or not publish_message blocks and
                # latches until exit.  We want to block if the user
                # has enabled latching (i.e. rate is none). It would
                # be good to reorganize this code more conceptually
                # but, for now, this is the best re-use of the
                # underlying methods.
                else:
                    publish_message(pub, msg_class, pub_args, None, bool(r) or once, verbose=verbose)
            except ValueError as e:
                sys.stderr.write("%s\n"%str(e))
                break
        if r is not None:
            r.sleep()
        if rospy.is_shutdown() or once:
            break

def stdin_yaml_arg():
    """
    Iterate over YAML documents in stdin
    :returns: for next list of arguments on stdin. Iterator returns a list of args for each call, ``iterator``
    """
    import yaml
    from select import select
    from select import error as select_error
    try:
        arg = 'x'
        rlist = [sys.stdin]
        wlist = xlist = []
        while not rospy.is_shutdown() and arg != '\n':
            buff = ''
            while arg != '' and arg.strip() != '---' and not rospy.is_shutdown():
                val, _, _ = select(rlist, wlist, xlist, 1.0)
                if not val:
                    continue
                # sys.stdin.readline() returns empty string on EOF
                arg = sys.stdin.readline() 
                if arg != '' and arg.strip() != '---':
                    buff = buff + arg

            if arg.strip() == '---': # End of document
                try:
                    loaded = yaml.safe_load(buff.rstrip())
                except Exception as e:
                    sys.stderr.write("Invalid YAML: %s\n"%str(e))
                if loaded is not None:
                    yield loaded
            elif arg == '': #EOF
                # we don't yield the remaining buffer in this case
                # because we don't want to publish partial inputs
                return
            
            arg = 'x' # reset

    except select_error:
        return # most likely ctrl-c interrupt
    
def _rostopic_cmd_list(argv):
    """
    Command-line parsing for 'rostopic list' command.
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog list [/namespace]", prog=NAME)
    parser.add_option("-b", "--bag",
                      dest="bag", default=None,
                      help="list topics in .bag file", metavar="BAGFILE")
    parser.add_option("-v", "--verbose",
                      dest="verbose", default=False,action="store_true",
                      help="list full details about each topic")
    parser.add_option("-p",
                      dest="publishers", default=False,action="store_true",
                      help="list only publishers")
    parser.add_option("-s",
                      dest="subscribers", default=False,action="store_true",
                      help="list only subscribers")
    parser.add_option("--host", dest="hostname", default=False, action="store_true",
                      help="group by host name")

    (options, args) = parser.parse_args(args)
    topic = None

    if len(args) == 1:
        topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    elif len(args) > 1:
        parser.error("you may only specify one input topic")
    if options.bag:
        if options.subscribers: 
            parser.error("-s option is not valid with bags")
        elif options.publishers:
            parser.error("-p option is not valid with bags")
        elif options.hostname:
            parser.error("--host option is not valid with bags")
        _rostopic_list_bag(options.bag, topic)
    else:
        if options.subscribers and options.publishers:
            parser.error("you may only specify one of -p, -s")

        exitval = _rostopic_list(topic, verbose=options.verbose, subscribers_only=options.subscribers, publishers_only=options.publishers, group_by_host=options.hostname) or 0
        if exitval != 0:
            sys.exit(exitval)

def _rostopic_cmd_info(argv):
    """
    Command-line parsing for 'rostopic info' command.
    """
    args = argv[2:]
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog info /topic", prog=NAME)
    (options, args) = parser.parse_args(args)

    if len(args) == 0:
        parser.error("you must specify a topic name")
    elif len(args) > 1:
        parser.error("you may only specify one topic name")
            
    topic = rosgraph.names.script_resolve_name('rostopic', args[0])
    exitval = _rostopic_info(topic) or 0
    if exitval != 0:
        sys.exit(exitval)
            
def _fullusage():
    print("""rostopic is a command-line tool for printing information about ROS Topics.

Commands:
\trostopic bw\tdisplay bandwidth used by topic
\trostopic delay\tdisplay delay of topic from timestamp in header
\trostopic echo\tprint messages to screen
\trostopic find\tfind topics by type
\trostopic hz\tdisplay publishing rate of topic    
\trostopic info\tprint information about active topic
\trostopic list\tlist active topics
\trostopic pub\tpublish data to topic
\trostopic type\tprint topic or field type

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))

def rostopicmain(argv=None):
    import rosbag
    if argv is None:
        argv=sys.argv
    # filter out remapping arguments in case we are being invoked via roslaunch
    argv = rospy.myargv(argv)
    
    # process argv
    if len(argv) == 1:
        _fullusage()
    try:
        command = argv[1]
        if command == 'echo':
            _rostopic_cmd_echo(argv)
        elif command == 'hz':
            _rostopic_cmd_hz(argv)
        elif command == 'type':
            _rostopic_cmd_type(argv)
        elif command == 'list':
            _rostopic_cmd_list(argv)
        elif command == 'info':
            _rostopic_cmd_info(argv)
        elif command == 'pub':
            _rostopic_cmd_pub(argv)
        elif command == 'bw':
            _rostopic_cmd_bw(argv)
        elif command == 'find':
            _rostopic_cmd_find(argv)
        elif command == 'delay':
            _rostopic_cmd_delay(argv)
        else:
            _fullusage()
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")
        sys.exit(1)
    except rosbag.ROSBagException as e:
        sys.stderr.write("ERROR: unable to use bag file: %s\n"%str(e))
        sys.exit(1)
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)
    except ROSTopicException as e:
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)
    except KeyboardInterrupt: pass
