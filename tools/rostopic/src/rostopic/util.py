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

from __future__ import division, print_function

from operator import itemgetter
from roslib.message import get_message_class
import rosgraph
import rospy
from rostopic.exceptions import ROSTopicIOException
import socket


def check_master():
    """
    Make sure that master is available
    :raises: :exc:`ROSTopicException` If unable to successfully communicate with master
    """
    try:
        rosgraph.Master('/rostopic').getPid()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

def master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

def sleep(duration):
    rospy.rostime.wallsleep(duration)

def _get_nested_attribute(msg, nested_attributes):
    value = msg
    for attr in nested_attributes.split('/'):
        value = getattr(value, attr)
    return value

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

def _get_topic_type(topic):
    """
    subroutine for getting the topic type
    :returns: topic type, real topic name and fn to evaluate the message instance
    if the topic points to a field within a topic, e.g. /rosout/msg, ``(str, str, fn)``
    """
    try:
        val = master_get_topic_types(rosgraph.Master('/rostopic'))
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
            msg_class = get_message_class(t_type)
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
                sleep(0.1)
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
    msg_class = get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?" % topic_type)
    return msg_class, real_topic, msg_eval
