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

def _rostopic_hz(topics, window_size=-1, filter_expr=None, use_wtime=False):
    """
    Periodically print the publishing rate of a topic to console until
    shutdown
    :param topics: topic names, ``list`` of ``str``
    :param window_size: number of messages to average over, -1 for infinite, ``int``
    :param filter_expr: Python filter expression that is called with m, the message instance
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
            rospy.Subscriber(real_topic, msg_class, rt.callback_hz, callback_args=topic)
        else:
            rospy.Subscriber(real_topic, rospy.AnyMsg, rt.callback_hz, callback_args=topic)
        print("subscribed to [%s]" % real_topic)

    if rospy.get_param('use_sim_time', False):
        print("WARNING: may be using simulated time",file=sys.stderr)

    while not rospy.is_shutdown():
        _sleep(1.0)
        rt.print_hz(topics)

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
                      help="calculates rate using wall time which can be helpful when clock isnt published during simulation")

    (options, args) = parser.parse_args(args)
    if len(args) == 0:
        parser.error("topic must be specified")
    try:
        if options.window_size != -1:
            import string
            window_size = string.atoi(options.window_size)
        else:
            window_size = options.window_size
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
                 use_wtime=options.use_wtime)
