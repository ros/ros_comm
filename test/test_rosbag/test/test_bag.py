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
# test_bag.py

import heapq
import os
import shutil
import sys
import time
import unittest

import genpy

import rosbag
from rosbag import bag
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String

class TestRosbag(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_opening_stream_works(self):
        f = open('/tmp/test_opening_stream_works.bag', 'wb')
        with rosbag.Bag(f, 'w') as b:
            for i in range(10):
                msg = Int32()
                msg.data = i
                b.write('/int', msg)
        
        f = open('/tmp/test_opening_stream_works.bag', 'rb')
        b = rosbag.Bag(f, 'r')
        self.assert_(len(list(b.read_messages())) == 10)
        b.close()

    def test_invalid_bag_arguments_fails(self):
        f = '/tmp/test_invalid_bad_arguments_fails.bag'
        
        def fn1(): rosbag.Bag('')
        def fn2(): rosbag.Bag(None)
        def fn3(): rosbag.Bag(f, 'z')        
        def fn4(): rosbag.Bag(f, 'r', compression='foobar')
        def fn5(): rosbag.Bag(f, 'r', chunk_threshold=-1000)
        for fn in [fn1, fn2, fn3, fn4, fn5]:
            self.failUnlessRaises(ValueError, fn)

    def test_io_on_close_fails(self):
        def fn():
            b = rosbag.Bag('/tmp/test_io_close_fails.bag', 'w')
            b.close()
            size = b.size()
        self.failUnlessRaises(ValueError, fn)
        
    def test_write_invalid_message_fails(self):
        def fn():
            with rosbag.Bag('/tmp/test_write_invalid_message_fails.bag', 'w') as b:
                b.write(None, None, None)
        self.failUnlessRaises(ValueError, fn)

    def test_simple_write_uncompressed_works(self):
        with rosbag.Bag('/tmp/test_simple_write_uncompressed_works.bag', 'w') as b:
            msg_count = 0
            for i in range(5, 0, -1):
                msg = Int32()
                msg.data = i
                t = genpy.Time.from_sec(i)
                b.write('/ints' + str(i), msg, t)
                msg_count += 1

        msgs = list(rosbag.Bag('/tmp/test_simple_write_uncompressed_works.bag').read_messages())
        
        self.assert_(len(msgs) == msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

        for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
            self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_writing_nonchronological_works(self):
        with rosbag.Bag('/tmp/test_writing_nonchronological_works.bag', 'w') as b:
            msg_count = 0
            for i in range(5, 0, -1):
                msg = Int32()
                msg.data = i
                t = genpy.Time.from_sec(i)
                b.write('/ints', msg, t)
                msg_count += 1

        msgs = list(rosbag.Bag('/tmp/test_writing_nonchronological_works.bag').read_messages())
        
        self.assert_(len(msgs) == msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

        for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
            self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_large_write_works(self):
        for compression in [rosbag.Compression.NONE, rosbag.Compression.BZ2]:
            with rosbag.Bag('/tmp/test_large_write_works.bag', 'w', compression=compression) as b:
                msg_count = 0
                for i in range(10000):
                    msg = Int32()
                    msg.data = i
                    t = genpy.Time.from_sec(i)
                    b.write('/ints', msg, t)
                    msg_count += 1

            msgs = list(rosbag.Bag('/tmp/test_large_write_works.bag').read_messages())

            self.assertEquals(len(msgs), msg_count, 'not all messages written: expected %d, got %d' % (msg_count, len(msgs)))

            for (_, _, t1), (_, _, t2) in zip(msgs, msgs[1:]):
                self.assert_(t1 < t2, 'messages returned unordered: got timestamp %s before %s' % (str(t1), str(t2)))

    def test_get_messages_time_range_works(self):
        with rosbag.Bag('/tmp/test_get_messages_time_range_works.bag', 'w') as b:
            for i in range(30):
                msg = Int32()
                msg.data = i
                t = genpy.Time.from_sec(i)
                b.write('/ints', msg, t)
        
        start_time = genpy.Time.from_sec(3)
        end_time = genpy.Time.from_sec(7)
        msgs = list(rosbag.Bag('/tmp/test_get_messages_time_range_works.bag').read_messages(topics='/ints', start_time=start_time, end_time=end_time))

        self.assertEquals(len(msgs), 5)
        
    def test_get_messages_filter_works(self):
        with rosbag.Bag('/tmp/test_get_messages_filter_works.bag', 'w') as b:
            for i in range(30):
                msg = Int32()
                msg.data = i
                t = genpy.Time.from_sec(i)
                b.write('/ints' + str(i), msg, t)

        def filter(topic, datatype, md5sum, msg_def, header):
            return '5' in topic and datatype == Int32._type and md5sum == Int32._md5sum and msg_def == Int32._full_text

        self.assertEquals(len(list(rosbag.Bag('/tmp/test_get_messages_filter_works.bag').read_messages(connection_filter=filter))), 3)

    def test_rosbag_filter(self):
        inbag_filename  = '/tmp/test_rosbag_filter__1.bag'
        outbag_filename = '/tmp/test_rosbag_filter__2.bag'
        
        with rosbag.Bag(inbag_filename, 'w') as b:
            for i in range(30):
                msg = Int32()
                msg.data = i
                t = genpy.Time.from_sec(i)
                b.write('/ints' + str(i), msg, t)

        expression = "(int(t.secs) == m.data) and (topic == '/ints' + str(m.data)) and (m.data >= 15 and m.data < 20)"

        os.system('rosbag filter %s %s "%s"' % (inbag_filename, outbag_filename, expression))

        msgs = list(rosbag.Bag(outbag_filename).read_messages())

        self.assertEquals(len(msgs), 5)

    def test_reindex_works(self):
        fn = '/tmp/test_reindex_works.bag'
        
        chunk_threshold = 1024

        with rosbag.Bag(fn, 'w', chunk_threshold=chunk_threshold) as b:
            for i in range(100):
                for j in range(5):
                    msg = Int32()
                    msg.data = i
                    b.write('/topic%d' % j, msg)
            file_header_pos = b._file_header_pos

        start_index = 4117 + chunk_threshold * 2 + int(chunk_threshold / 2)

        trunc_filename   = '%s.trunc%s'   % os.path.splitext(fn)
        reindex_filename = '%s.reindex%s' % os.path.splitext(fn)

        for trunc_index in range(start_index, start_index + chunk_threshold):
            shutil.copy(fn, trunc_filename)
            
            with open(trunc_filename, 'r+b') as f:
                f.seek(file_header_pos)
                header = {
                    'op':          bag._pack_uint8(bag._OP_FILE_HEADER),
                    'index_pos':   bag._pack_uint64(0),
                    'conn_count':  bag._pack_uint32(0),
                    'chunk_count': bag._pack_uint32(0)
                }
                bag._write_record(f, header, padded_size=bag._FILE_HEADER_LENGTH)
                f.truncate(trunc_index)

            shutil.copy(trunc_filename, reindex_filename)
       
            try:
                b = rosbag.Bag(reindex_filename, 'a', allow_unindexed=True)
            except Exception as ex:
                pass
            for done in b.reindex():
                pass
            b.close()

            msgs = list(rosbag.Bag(reindex_filename, 'r'))

    def test_future_version_works(self):
        fn = '/tmp/test_future_version_2.1.bag'

        with rosbag.Bag(fn, 'w', chunk_threshold=256) as b:
            for i in range(10):
                msg = Int32()
                msg.data = i
                b.write('/int', msg)

                header = { 'op': bag._pack_uint8(max(bag._OP_CODES.keys()) + 1) }
                data = 'ABCDEFGHI123456789'
                bag._write_record(b._file, header, data)

            b._file.seek(0)
            data = '#ROSBAG V%d.%d\n' % (int(b._version / 100), (b._version % 100) + 1)  # increment the minor version
            data = data.encode()
            b._file.write(data)
            b._file.seek(0, os.SEEK_END)

        with rosbag.Bag(fn) as b:
            for topic, msg, t in b:
                pass
            
    def test_get_message_count(self):
        fn = '/tmp/test_get_message_count.bag'
        with rosbag.Bag(fn, mode='w') as bag:
            for i in xrange(100):
                bag.write("/test_bag", Int32(data=i))
                bag.write("/test_bag", String(data='also'))
                bag.write("/test_bag/more", String(data='alone'))
                
        with rosbag.Bag(fn) as bag:
            self.assertEquals(bag.get_message_count(), 300)
            self.assertEquals(bag.get_message_count(topic_filters='/test_bag'), 200)
            self.assertEquals(bag.get_message_count(topic_filters=['/test_bag', '/test_bag/more']), 300)
            self.assertEquals(bag.get_message_count(topic_filters=['/none']), 0)
        
    def test_get_compression_info(self):
        fn = '/tmp/test_get_compression_info.bag'
        
        # No Compression
        with rosbag.Bag(fn, mode='w') as bag:
            for i in xrange(100):
                bag.write("/test_bag", Int32(data=i))
                
        with rosbag.Bag(fn) as bag:
            info = bag.get_compression_info()
            self.assertEquals(info.compression, rosbag.Compression.NONE)
            # 167 Bytes of overhead, 50 Bytes per Int32.
            self.assertEquals(info.uncompressed, 5166)
            self.assertEquals(info.compressed, 5166)
        
        with rosbag.Bag(fn, mode='w', compression=rosbag.Compression.BZ2) as bag:
            for i in xrange(100):
                bag.write("/test_bag", Int32(data=i))
                
        with rosbag.Bag(fn) as bag:
            info = bag.get_compression_info()
            self.assertEquals(info.compression, rosbag.Compression.BZ2)
            self.assertEquals(info.uncompressed, 5166)
            
            # the value varies each run, I suspect based on rand, but seems
            # to generally be around 960 to 980 on my comp
            self.assertLess(info.compressed, 1000)
            self.assertGreater(info.compressed, 800)
        
    def test_get_time(self):
        fn = '/tmp/test_get_time.bag'
        
        with rosbag.Bag(fn, mode='w') as bag:
            for i in xrange(100):
                bag.write("/test_bag", Int32(data=i), t=genpy.Time.from_sec(i))
                
        with rosbag.Bag(fn) as bag:
            start_stamp = bag.get_start_time()
            end_stamp = bag.get_end_time()
            
            self.assertEquals(start_stamp, 0.0)
            self.assertEquals(end_stamp, 99.0)

    def test_get_time_empty_bag(self):
        """Test for issue #657"""
        fn = '/tmp/test_get_time_emtpy_bag.bag'

        with rosbag.Bag(fn, mode='w') as bag:
            self.assertRaisesRegexp(rosbag.ROSBagException,
                                    'Bag contains no message',
                                    bag.get_start_time)
            self.assertRaisesRegexp(rosbag.ROSBagException,
                                    'Bag contains no message',
                                    bag.get_end_time)

    def test_get_type_and_topic_info(self):
        fn = '/tmp/test_get_type_and_topic_info.bag'
        topic_1 = "/test_bag"
        topic_2 = "/test_bag/more"
        with rosbag.Bag(fn, mode='w') as bag:
            for i in xrange(100):
                bag.write(topic_1, Int32(data=i))
                bag.write(topic_1, String(data='also'))
                bag.write(topic_2, String(data='alone'))
                
        with rosbag.Bag(fn) as bag:
            msg_types, topics = bag.get_type_and_topic_info()
            self.assertEquals(len(msg_types), 2)
            self.assertTrue("std_msgs/Int32" in msg_types)
            self.assertTrue("std_msgs/String" in msg_types)
            self.assertEquals(len(topics), 2)
            self.assertTrue(topic_1 in topics)
            self.assertTrue(topic_2 in topics)
            
            self.assertEquals(topics[topic_1].message_count, 200)
            self.assertEquals(topics[topic_1].msg_type, "std_msgs/Int32")
            self.assertEquals(topics[topic_2].message_count, 100)
            self.assertEquals(topics[topic_2].msg_type, "std_msgs/String")
            
            #filter on topic 1
            msg_types, topics = bag.get_type_and_topic_info(topic_1)
            
            # msg_types should be unaffected by the filter
            self.assertEquals(len(msg_types), 2)
            self.assertTrue("std_msgs/Int32" in msg_types)
            self.assertTrue("std_msgs/String" in msg_types)            
            
            self.assertEquals(len(topics), 1)
            self.assertTrue(topic_1 in topics)
            
            self.assertEquals(topics[topic_1].message_count, 200)
            self.assertEquals(topics[topic_1].msg_type, "std_msgs/Int32")
            
            #filter on topic 2
            msg_types, topics = bag.get_type_and_topic_info(topic_2)
            
            # msg_types should be unaffected by the filter
            self.assertEquals(len(msg_types), 2)
            self.assertTrue("std_msgs/Int32" in msg_types)
            self.assertTrue("std_msgs/String" in msg_types)            
            
            self.assertEquals(len(topics), 1)
            self.assertTrue(topic_2 in topics)
            
            self.assertEquals(topics[topic_2].message_count, 100)
            self.assertEquals(topics[topic_2].msg_type, "std_msgs/String")
            
            #filter on missing topic
            msg_types, topics = bag.get_type_and_topic_info("/none")
            
            # msg_types should be unaffected by the filter
            self.assertEquals(len(msg_types), 2)
            self.assertTrue("std_msgs/Int32" in msg_types)
            self.assertTrue("std_msgs/String" in msg_types)            
            
            # topics should be empty
            self.assertEquals(len(topics), 0)
        
    def _print_bag_records(self, fn):
        with open(fn) as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(0)

            version_line = f.readline().rstrip()
            print(version_line)

            while f.tell() < size:
                header = bag._read_header(f)
                op = bag._read_uint8_field(header, 'op')
                data = bag._read_record_data(f)

                print(bag._OP_CODES.get(op, op))

if __name__ == '__main__':
    import rostest
    PKG='rosbag'
    rostest.run(PKG, 'TestRosbag', TestRosbag, sys.argv)
