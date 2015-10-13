# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Message Filter Objects
======================
"""

import itertools
import threading
import rospy

class SimpleFilter:

    def __init__(self):
        self.callbacks = {}

    def registerCallback(self, cb, *args):
        """
        Register a callback function `cb` to be called when this filter
        has output.
        The filter calls the function ``cb`` with a filter-dependent list of arguments,
        followed by the call-supplied arguments ``args``.
        """

        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))

class Subscriber(SimpleFilter):
    
    """
    ROS subscription filter.  Identical arguments as :class:`rospy.Subscriber`.

    This class acts as a highest-level filter, simply passing messages
    from a ROS subscription through to the filters which have connected
    to it.
    """
    def __init__(self, *args, **kwargs):
        SimpleFilter.__init__(self)
        self.topic = args[0]
        kwargs['callback'] = self.callback
        self.sub = rospy.Subscriber(*args, **kwargs)

    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic

    def unregister(self):
        self.sub.unregister()

class Cache(SimpleFilter):

    """
    Stores a time history of messages.

    Given a stream of messages, the most recent ``cache_size`` messages
    are cached in a ring buffer, from which time intervals of the cache
    can then be retrieved by the client.
    """

    def __init__(self, f, cache_size = 1):
        SimpleFilter.__init__(self)
        self.connectInput(f)
        self.cache_size = cache_size
        # Array to store messages
        self.cache_msgs = []
        # Array to store msgs times, auxiliary structure to facilitate
        # sorted insertion
        self.cache_times = []

    def connectInput(self, f):
        self.incoming_connection = f.registerCallback(self.add)

    def add(self, msg):
        # Cannot use message filters with non-stamped messages
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            rospy.logwarn("Cannot use message filters with non-stamped messages")
            return

        # Insert sorted
        stamp = msg.header.stamp
        self.cache_times.append(stamp)
        self.cache_msgs.append(msg)

        # Implement a ring buffer, discard older if oversized
        if (len(self.cache_msgs) > self.cache_size):
            del self.cache_msgs[0]
            del self.cache_times[0]

        # Signal new input
        self.signalMessage(msg)

    def getInterval(self, from_stamp, to_stamp):
        """Query the current cache content between from_stamp to to_stamp."""
        assert from_stamp <= to_stamp
        return [m for m in self.cache_msgs
                if m.header.stamp >= from_stamp and m.header.stamp <= to_stamp]

    def getElemAfterTime(self, stamp):
        """Return the oldest element after or equal the passed time stamp."""
        newer = [m for m in self.cache_msgs if m.header.stamp >= stamp]
        if not newer:
            return None
        return newer[0]

    def getElemBeforeTime(self, stamp):
        """Return the newest element before or equal the passed time stamp."""
        older = [m for m in self.cache_msgs if m.header.stamp <= stamp]
        if not older:
            return None
        return older[-1]

    def getLastestTime(self):
        """Return the newest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[-1]

    def getOldestTime(self):
        """Return the oldest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[0]


class TimeSynchronizer(SimpleFilter):

    """
    Synchronizes messages by their timestamps.

    :class:`TimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. TimeSynchronizer
    listens on multiple input message filters ``fs``, and invokes the callback
    when it has a collection of messages with matching timestamps.

    The signature of the callback function is::

        def callback(msg1, ... msgN):

    where N is the number of input message filters, and each message is
    the output of the corresponding filter in ``fs``.
    The required ``queue size`` parameter specifies how many sets of
    messages it should store from each input filter (by timestamp)
    while waiting for messages to arrive and complete their "set".
    """

    def __init__(self, fs, queue_size):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [f.registerCallback(self.add, q) for (f, q) in zip(fs, self.queues)]

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # common is the set of timestamps that occur in all queues
        common = reduce(set.intersection, [set(q) for q in self.queues])
        for t in sorted(common):
            # msgs is list of msgs (one from each queue) with stamp t
            msgs = [q[t] for q in self.queues]
            self.signalMessage(*msgs)
            for q in self.queues:
                del q[t]
        self.lock.release()

class ApproximateTimeSynchronizer(TimeSynchronizer):

    """
    Approximately synchronizes messages by their timestamps.

    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. The API is the same as TimeSynchronizer
    except for an extra `slop` parameter in the constructor that defines the delay (in seconds)
    with which messages can be synchronized
    """

    def __init__(self, fs, queue_size, slop):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = rospy.Duration.from_sec(slop)

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        for vv in itertools.product(*[list(q.keys()) for q in self.queues]):
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t]
        self.lock.release()
