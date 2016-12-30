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


SUBSCRIBER_TIMEOUT = 5.
def wait_for_subscriber(pub, timeout):
    timeout_t = time.time() + timeout
    while pub.get_num_connections() == 0 and timeout_t > time.time():
        _sleep(0.01)


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

def stdin_publish(pub, msg_class, rate, once, filename, verbose):
    """
    :param filename: name of file to read from instead of stdin, or ``None``, ``str``
    """
    if filename:
        iterator = file_yaml_arg(filename)
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
                # we use 'bool(r) or once' for the once value, which
                # controls whether or not publish_message blocks and
                # latches until exit.  We want to block if the user
                # has enabled latching (i.e. rate is none). It would
                # be good to reorganize this code more conceptually
                # but, for now, this is the best re-use of the
                # underlying methods.
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
                    loaded = yaml.load(buff.rstrip())
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

def _resource_name_package(name):
    """
    pkg/typeName -> pkg, typeName -> None

    :param name: package resource name, e.g. 'std_msgs/String', ``str``
    :returns: package name of resource, ``str``
    """
    if not '/' in name:
        return None
    return name[:name.find('/')]

def create_publisher(topic_name, topic_type, latch):
    """
    Create rospy.Publisher instance from the string topic name and
    type. This is a powerful method as it allows creation of
    rospy.Publisher and Message instances using the topic and type
    names. This enables more dynamic publishing from Python programs.

    :param topic_name: name of topic, ``str``
    :param topic_type: name of topic type, ``str``
    :param latch: latching topic, ``bool``
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
    rospy.init_node('rostopic', anonymous=True, disable_rosout=True, disable_rostime=True)
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
            parser.error("args confict with -p setting")
    if len(args) > 2 and options.file:
        parser.error("args confict with -f setting")
    topic_name, topic_type = args[0], args[1]

    # type-case using YAML
    try:
        pub_args = []
        for arg in args[2:]:
            pub_args.append(yaml.load(arg))
    except Exception as e:
        parser.error("Argument error: "+str(e))

    # make sure master is online. we wait until after we've parsed the
    # args to do this so that syntax errors are reported first
    _check_master()

    # if no rate, or explicit latch, we latch
    latch = (rate == None) or options.latch
    pub, msg_class = create_publisher(topic_name, topic_type, latch)

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
        stdin_publish(pub, msg_class, rate, options.once, options.file, options.verbose)
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
                data = yaml.load_all(f)
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

def param_publish_once(pub, msg_class, param_name, verbose):
    if not rospy.has_param(param_name):
        raise ROSTopicException("parameter does not exist: %s"%(param_name))
    pub_args = rospy.get_param(param_name)
    argv_publish(pub, msg_class, pub_args, None, True, verbose)
