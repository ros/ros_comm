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

from rostopic import NAME

import genpy
import rosgraph
import roslib.message
import rospy
import traceback


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
    if type_ in (bool, int, float) or \
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
        if type0 in (bool, int, float) or \
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
    elif type_ in (int, float) or \
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
        elif type0 in (int, float) or \
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
