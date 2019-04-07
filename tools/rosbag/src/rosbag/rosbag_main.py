# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

from __future__ import print_function

import optparse
import os
import shutil
import signal
import subprocess
import sys
import time
try:
    from UserDict import UserDict  # Python 2.x
except ImportError:
    from collections import UserDict  # Python 3.x

import roslib.message
import roslib.packages

from .bag import Bag, Compression, ROSBagException, ROSBagFormatException, ROSBagUnindexedException, ROSBagEncryptNotSupportedException, ROSBagEncryptException
from .migration import MessageMigrator, fixbag2, checkbag

def print_trans(old, new, indent):
    from_txt = '%s [%s]' % (old._type, old._md5sum)
    if new is not None:
        to_txt= '%s [%s]' % (new._type, new._md5sum)
    else:
        to_txt = 'Unknown'
    print('    ' * indent + ' * From: %s' % from_txt)
    print('    ' * indent + '   To:   %s' % to_txt)

def handle_split(option, opt_str, value, parser):
    parser.values.split = True
    if len(parser.rargs) > 0 and parser.rargs[0].isdigit():
        print("Use of \"--split <MAX_SIZE>\" has been deprecated.  Please use --split --size <MAX_SIZE> or --split --duration <MAX_DURATION>", file=sys.stderr)
        parser.values.size = int(parser.rargs.pop(0))


def _stop_process(signum, frame, old_handler, process):
    process.terminate()
    if old_handler:
        old_handler(signum, frame)


def record_cmd(argv):
    parser = optparse.OptionParser(usage="rosbag record TOPIC1 [TOPIC2 TOPIC3 ...]",
                                   description="Record a bag file with the contents of specified topics.",
                                   formatter=optparse.IndentedHelpFormatter())

    parser.add_option("-a", "--all",           dest="all",           default=False, action="store_true",          help="record all topics")
    parser.add_option("-e", "--regex",         dest="regex",         default=False, action="store_true",          help="match topics using regular expressions")
    parser.add_option("-p", "--publish",       dest="publish",       default=False, action="store_true",          help="publish a msg when the record begin")
    parser.add_option("-x", "--exclude",       dest="exclude_regex", default="",    action="store",               help="exclude topics matching the follow regular expression (subtracts from -a or regex)")
    parser.add_option("-q", "--quiet",         dest="quiet",         default=False, action="store_true",          help="suppress console output")
    parser.add_option("-o", "--output-prefix", dest="prefix",        default=None,  action="store",               help="prepend PREFIX to beginning of bag name (name will always end with date stamp)")
    parser.add_option("-O", "--output-name",   dest="name",          default=None,  action="store",               help="record to bag with name NAME.bag")
    parser.add_option(      "--split",         dest="split",         default=False, callback=handle_split, action="callback",    help="split the bag when maximum size or duration is reached")
    parser.add_option(      "--max-splits",    dest="max_splits",                   type='int',   action="store", help="Keep a maximum of N bag files, when reaching the maximum erase the oldest one to keep a constant number of files.", metavar="MAX_SPLITS")
    parser.add_option(      "--size",          dest="size",                         type='int',   action="store", help="record a bag of maximum size SIZE MB. (Default: infinite)", metavar="SIZE")
    parser.add_option(      "--duration",      dest="duration",                     type='string',action="store", help="record a bag of maximum duration DURATION in seconds, unless 'm', or 'h' is appended.", metavar="DURATION")
    parser.add_option("-b", "--buffsize",      dest="buffsize",      default=256,   type='int',   action="store", help="use an internal buffer of SIZE MB (Default: %default, 0 = infinite)", metavar="SIZE")
    parser.add_option("--chunksize",           dest="chunksize",     default=768,   type='int',   action="store", help="Advanced. Record to chunks of SIZE KB (Default: %default)", metavar="SIZE")
    parser.add_option("-l", "--limit",         dest="num",           default=0,     type='int',   action="store", help="only record NUM messages on each topic")
    parser.add_option(      "--node",          dest="node",          default=None,  type='string',action="store", help="record all topics subscribed to by a specific node")
    parser.add_option("-j", "--bz2",           dest="compression",   default=None,  action="store_const", const='bz2', help="use BZ2 compression")
    parser.add_option("--lz4",                 dest="compression",                  action="store_const", const='lz4', help="use LZ4 compression")
    parser.add_option("--tcpnodelay",          dest="tcpnodelay",                   action="store_true",          help="Use the TCP_NODELAY transport hint when subscribing to topics.")
    parser.add_option("--udp",                 dest="udp",                          action="store_true",          help="Use the UDP transport hint when subscribing to topics.")

    (options, args) = parser.parse_args(argv)

    if len(args) == 0 and not options.all and not options.node:
        parser.error("You must specify a topic name or else use the '-a' option.")

    if options.prefix is not None and options.name is not None:
        parser.error("Can't set both prefix and name.")

    recordpath = roslib.packages.find_node('rosbag', 'record')
    if not recordpath:
        parser.error("Cannot find rosbag/record executable")
    cmd = [recordpath[0]]

    cmd.extend(['--buffsize',  str(options.buffsize)])
    cmd.extend(['--chunksize', str(options.chunksize)])

    if options.num != 0:      cmd.extend(['--limit', str(options.num)])
    if options.quiet:         cmd.extend(["--quiet"])
    if options.prefix:        cmd.extend(["-o", options.prefix])
    if options.name:          cmd.extend(["-O", options.name])
    if options.exclude_regex: cmd.extend(["--exclude", options.exclude_regex])
    if options.all:           cmd.extend(["--all"])
    if options.regex:         cmd.extend(["--regex"])
    if options.publish:       cmd.extend(["--publish"])
    if options.compression:   cmd.extend(["--%s" % options.compression])
    if options.split:
        if not options.duration and not options.size:
            parser.error("Split specified without giving a maximum duration or size")
        cmd.extend(["--split"])
        if options.max_splits:
            cmd.extend(["--max-splits", str(options.max_splits)])
    if options.duration:    cmd.extend(["--duration", options.duration])
    if options.size:        cmd.extend(["--size", str(options.size)])
    if options.node:
        cmd.extend(["--node", options.node])
    if options.tcpnodelay:  cmd.extend(["--tcpnodelay"])
    if options.udp:         cmd.extend(["--udp"])

    cmd.extend(args)

    old_handler = signal.signal(
        signal.SIGTERM,
        lambda signum, frame: _stop_process(signum, frame, old_handler, process)
    )
    # Better way of handling it than os.execv
    # This makes sure stdin handles are passed to the process.
    process = subprocess.Popen(cmd)
    process.wait()


def info_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag info [options] BAGFILE1 [BAGFILE2 BAGFILE3 ...]',
                                   description='Summarize the contents of one or more bag files.')
    parser.add_option('-y', '--yaml', dest='yaml', default=False, action='store_true', help='print information in YAML format')
    parser.add_option('-k', '--key',  dest='key',  default=None,  action='store',      help='print information on the given key')
    parser.add_option(      '--freq', dest='freq', default=False, action='store_true', help='display topic message frequency statistics')
    (options, args) = parser.parse_args(argv)

    if len(args) == 0:
        parser.error('You must specify at least 1 bag file.')
    if options.key and not options.yaml:
        parser.error('You can only specify key when printing in YAML format.')

    for i, arg in enumerate(args):
        try:
            b = Bag(arg, 'r', skip_index=not options.freq)
            if options.yaml:
                info = b._get_yaml_info(key=options.key)
                if info is not None:
                    print(info)
            else:
                print(b)
            b.close()
            if i < len(args) - 1:
                print('---')
        
        except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
            print('ERROR: %s' % str(ex), file=sys.stderr)
        except ROSBagUnindexedException as ex:
            print('ERROR bag unindexed: %s.  Run rosbag reindex.' % arg,
                  file=sys.stderr)
            sys.exit(1)
        except ROSBagException as ex:
            print('ERROR reading %s: %s' % (arg, str(ex)), file=sys.stderr)
            sys.exit(1)
        except IOError as ex:
            print('ERROR reading %s: %s' % (arg, str(ex)), file=sys.stderr)
            sys.exit(1)


def handle_topics(option, opt_str, value, parser):
    topics = []
    for arg in parser.rargs:
        if arg[:2] == "--" and len(arg) > 2:
            break
        if arg[:1] == "-" and len(arg) > 1:
            break
        topics.append(arg)
    parser.values.topics.extend(topics)
    del parser.rargs[:len(topics)]


def handle_pause_topics(option, opt_str, value, parser):
    pause_topics = []
    for arg in parser.rargs:
        if arg[:2] == "--" and len(arg) > 2:
            break
        if arg[:1] == "-" and len(arg) > 1:
            break
        pause_topics.append(arg)
    parser.values.pause_topics.extend(pause_topics)
    del parser.rargs[:len(pause_topics)]


def play_cmd(argv):
    parser = optparse.OptionParser(usage="rosbag play BAGFILE1 [BAGFILE2 BAGFILE3 ...]",
                                   description="Play back the contents of one or more bag files in a time-synchronized fashion.")
    parser.add_option("-p", "--prefix",       dest="prefix",     default='',    type='str',          help="prefix all output topics")
    parser.add_option("-q", "--quiet",        dest="quiet",      default=False, action="store_true", help="suppress console output")
    parser.add_option("-i", "--immediate",    dest="immediate",  default=False, action="store_true", help="play back all messages without waiting")
    parser.add_option("--pause",              dest="pause",      default=False, action="store_true", help="start in paused mode")
    parser.add_option("--queue",              dest="queue",      default=100,     type='int', action="store", help="use an outgoing queue of size SIZE (defaults to %default)", metavar="SIZE")
    parser.add_option("--clock",              dest="clock",      default=False, action="store_true", help="publish the clock time")
    parser.add_option("--hz",                 dest="freq",       default=100,   type='float', action="store", help="use a frequency of HZ when publishing clock time (default: %default)", metavar="HZ")
    parser.add_option("-d", "--delay",        dest="delay",      default=0.2,   type='float', action="store", help="sleep SEC seconds after every advertise call (to allow subscribers to connect)", metavar="SEC")
    parser.add_option("-r", "--rate",         dest="rate",       default=1.0,   type='float', action="store", help="multiply the publish rate by FACTOR", metavar="FACTOR")
    parser.add_option("-s", "--start",        dest="start",      default=0.0,   type='float', action="store", help="start SEC seconds into the bag files", metavar="SEC")
    parser.add_option("-u", "--duration",     dest="duration",   default=None,  type='float', action="store", help="play only SEC seconds from the bag files", metavar="SEC")
    parser.add_option("--skip-empty",         dest="skip_empty", default=None,  type='float', action="store", help="skip regions in the bag with no messages for more than SEC seconds", metavar="SEC")
    parser.add_option("-l", "--loop",         dest="loop",       default=False, action="store_true", help="loop playback")
    parser.add_option("-k", "--keep-alive",   dest="keep_alive", default=False, action="store_true", help="keep alive past end of bag (useful for publishing latched topics)")
    parser.add_option("--try-future-version", dest="try_future", default=False, action="store_true", help="still try to open a bag file, even if the version number is not known to the player")
    parser.add_option("--topics", dest="topics", default=[],  callback=handle_topics, action="callback", help="topics to play back")
    parser.add_option("--pause-topics", dest="pause_topics", default=[],  callback=handle_pause_topics, action="callback", help="topics to pause on during playback")
    parser.add_option("--bags",  help="bags files to play back from")
    parser.add_option("--wait-for-subscribers",  dest="wait_for_subscribers", default=False, action="store_true", help="wait for at least one subscriber on each topic before publishing")
    parser.add_option("--rate-control-topic", dest="rate_control_topic", default='', type='str', help="watch the given topic, and if the last publish was more than <rate-control-max-delay> ago, wait until the topic publishes again to continue playback")
    parser.add_option("--rate-control-max-delay", dest="rate_control_max_delay", default=1.0, type='float', help="maximum time difference from <rate-control-topic> before pausing")

    (options, args) = parser.parse_args(argv)

    if options.bags:
        args.append(options.bags)

    if len(args) == 0:
        parser.error('You must specify at least 1 bag file to play back.')

    playpath = roslib.packages.find_node('rosbag', 'play')
    if not playpath:
        parser.error("Cannot find rosbag/play executable")
    cmd = [playpath[0]]

    if options.prefix:
        cmd.extend(["--prefix", str(options.prefix)])

    if options.quiet:      cmd.extend(["--quiet"])
    if options.pause:      cmd.extend(["--pause"])
    if options.immediate:  cmd.extend(["--immediate"])
    if options.loop:       cmd.extend(["--loop"])
    if options.keep_alive: cmd.extend(["--keep-alive"])
    if options.try_future: cmd.extend(["--try-future-version"])
    if options.wait_for_subscribers: cmd.extend(["--wait-for-subscribers"])

    if options.clock:
        cmd.extend(["--clock", "--hz", str(options.freq)])

    cmd.extend(['--queue', str(options.queue)])
    cmd.extend(['--rate', str(options.rate)])
    cmd.extend(['--delay', str(options.delay)])
    cmd.extend(['--start', str(options.start)])
    if options.duration:
        cmd.extend(['--duration', str(options.duration)])
    if options.skip_empty:
        cmd.extend(['--skip-empty', str(options.skip_empty)])

    if options.topics:
        cmd.extend(['--topics'] + options.topics)

    if options.pause_topics:
        cmd.extend(['--pause-topics'] + options.pause_topics)

    # prevent bag files to be passed as --topics or --pause-topics
    if options.topics or options.pause_topics:
        cmd.extend(['--bags'])

    cmd.extend(args)

    if options.rate_control_topic:
        cmd.extend(['--rate-control-topic', str(options.rate_control_topic)])

    if options.rate_control_max_delay:
        cmd.extend(['--rate-control-max-delay', str(options.rate_control_max_delay)])

    old_handler = signal.signal(
        signal.SIGTERM,
        lambda signum, frame: _stop_process(signum, frame, old_handler, process)
    )
    # Better way of handling it than os.execv
    # This makes sure stdin handles are passed to the process.
    process = subprocess.Popen(cmd)
    process.wait()


def filter_cmd(argv):
    def expr_eval(expr):
        def eval_fn(topic, m, t):
            return eval(expr)
        return eval_fn

    parser = optparse.OptionParser(usage="""rosbag filter [options] INBAG OUTBAG EXPRESSION

EXPRESSION can be any Python-legal expression.

The following variables are available:
 * topic: name of topic
 * m: message
 * t: time of message (t.secs, t.nsecs)""",
                                   description='Filter the contents of the bag.')
    parser.add_option('-p', '--print', action='store', dest='verbose_pattern', default=None, metavar='PRINT-EXPRESSION', help='Python expression to print for verbose debugging. Uses same variables as filter-expression')

    options, args = parser.parse_args(argv)
    if len(args) == 0:
        parser.error('You must specify an in bag, an out bag, and an expression.')
    if len(args) == 1:
        parser.error('You must specify an out bag and an expression.')
    if len(args) == 2:
        parser.error("You must specify an expression.")
    if len(args) > 3:
        parser.error("Too many arguments.")

    inbag_filename, outbag_filename, expr = args

    if not os.path.isfile(inbag_filename):
        print('Cannot locate input bag file [%s]' % inbag_filename, file=sys.stderr)
        sys.exit(2)

    if os.path.realpath(inbag_filename) == os.path.realpath(outbag_filename):
        print('Cannot use same file as input and output [%s]' % inbag_filename, file=sys.stderr)
        sys.exit(3)

    filter_fn = expr_eval(expr)

    outbag = Bag(outbag_filename, 'w')
    
    try:
        inbag = Bag(inbag_filename)
    except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
        print('ERROR: %s' % str(ex), file=sys.stderr)
        return
    except ROSBagUnindexedException as ex:
        print('ERROR bag unindexed: %s.  Run rosbag reindex.' % inbag_filename, file=sys.stderr)
        sys.exit(1)

    try:
        meter = ProgressMeter(outbag_filename, inbag._uncompressed_size)
        total_bytes = 0
    
        if options.verbose_pattern:
            verbose_pattern = expr_eval(options.verbose_pattern)
    
            for topic, raw_msg, t, conn_header in inbag.read_messages(raw=True, return_connection_header=True):
                msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg
                msg = pytype()
                msg.deserialize(serialized_bytes)

                if filter_fn(topic, msg, t):
                    print('MATCH', verbose_pattern(topic, msg, t))
                    outbag.write(topic, msg, t, connection_header=conn_header)
                else:
                    print('NO MATCH', verbose_pattern(topic, msg, t))          

                total_bytes += len(serialized_bytes) 
                meter.step(total_bytes)
        else:
            for topic, raw_msg, t, conn_header in inbag.read_messages(raw=True, return_connection_header=True):
                msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg
                msg = pytype()
                msg.deserialize(serialized_bytes)

                if filter_fn(topic, msg, t):
                    outbag.write(topic, msg, t, connection_header=conn_header)

                total_bytes += len(serialized_bytes)
                meter.step(total_bytes)
        
        meter.finish()

    finally:
        inbag.close()
        outbag.close()

def fix_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag fix INBAG OUTBAG [EXTRARULES1 EXTRARULES2 ...]', description='Repair the messages in a bag file so that it can be played in the current system.')
    parser.add_option('-n', '--noplugins', action='store_true', dest='noplugins', help='do not load rulefiles via plugins')
    parser.add_option('--force', action='store_true', dest='force', help='proceed with migrations, even if not all rules defined')

    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must pass input and output bag files.')
    if len(args) < 2:
        parser.error('You must pass an output bag file.')

    inbag_filename  = args[0]
    outbag_filename = args[1]
    rules           = args[2:]   

    ext = os.path.splitext(outbag_filename)[1]
    if ext == '.bmr':
        parser.error('Input file should be a bag file, not a rule file.')
    if ext != '.bag':
        parser.error('Output file must be a bag file.')

    outname = outbag_filename + '.tmp'

    if os.path.exists(outbag_filename):
        if not os.access(outbag_filename, os.W_OK):
            print('Don\'t have permissions to access %s' % outbag_filename, file=sys.stderr)
            sys.exit(1)
    else:
        try:
            file = open(outbag_filename, 'w')
            file.close()
        except IOError as e:
            print('Cannot open %s for writing' % outbag_filename, file=sys.stderr)
            sys.exit(1)

    if os.path.exists(outname):
        if not os.access(outname, os.W_OK):
            print('Don\'t have permissions to access %s' % outname, file=sys.stderr)
            sys.exit(1)
    else:
        try:
            file = open(outname, 'w')
            file.close()
        except IOError as e:
            print('Cannot open %s for writing' % outname, file=sys.stderr)
            sys.exit(1)

    if options.noplugins is None:
        options.noplugins = False

    migrator = MessageMigrator(rules, plugins=not options.noplugins)
    
    try:
        migrations = fixbag2(migrator, inbag_filename, outname, options.force)
    except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
        print('ERROR: %s' % str(ex), file=sys.stderr)
        return
    except ROSBagUnindexedException as ex:
        print('ERROR bag unindexed: %s.  Run rosbag reindex.' % inbag_filename,
              file=sys.stderr)
        sys.exit(1)

    if len(migrations) == 0:
        os.rename(outname, outbag_filename)
        print('Bag migrated successfully.')
    else:
        print('Bag could not be migrated.  The following migrations could not be performed:')
        for m in migrations:
            print_trans(m[0][0].old_class, m[0][-1].new_class, 0)
            
            if len(m[1]) > 0:
                print('    %d rules missing:' % len(m[1]))
                for r in m[1]:
                    print_trans(r.old_class, r.new_class,1)
                    
        print('Try running \'rosbag check\' to create the necessary rule files or run \'rosbag fix\' with the \'--force\' option.')
        os.remove(outname)
        sys.exit(1)

def check_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag check BAG [-g RULEFILE] [EXTRARULES1 EXTRARULES2 ...]', description='Determine whether a bag is playable in the current system, or if it can be migrated.')
    parser.add_option('-g', '--genrules',  action='store',      dest='rulefile', default=None, help='generate a rulefile named RULEFILE')
    parser.add_option('-a', '--append',    action='store_true', dest='append',                 help='append to the end of an existing rulefile after loading it')
    parser.add_option('-n', '--noplugins', action='store_true', dest='noplugins',              help='do not load rulefiles via plugins')
    (options, args) = parser.parse_args(argv)

    if len(args) == 0:
        parser.error('You must specify a bag file to check.')
    if options.append and options.rulefile is None:
        parser.error('Cannot specify -a without also specifying -g.')
    if options.rulefile is not None:
        rulefile_exists = os.path.isfile(options.rulefile)
        if rulefile_exists and not options.append:
            parser.error('The file %s already exists.  Include -a if you intend to append.' % options.rulefile)
        if not rulefile_exists and options.append:
            parser.error('The file %s does not exist, and so -a is invalid.' % options.rulefile)
    
    if options.append:
        append_rule = [options.rulefile]
    else:
        append_rule = []

    # First check that the bag is not unindexed 
    try:
        Bag(args[0])
    except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
        print('ERROR: %s' % str(ex), file=sys.stderr)
        return
    except ROSBagUnindexedException as ex:
        print('ERROR bag unindexed: %s.  Run rosbag reindex.' % args[0], file=sys.stderr)
        sys.exit(1)

    mm = MessageMigrator(args[1:] + append_rule, not options.noplugins)

    migrations = checkbag(mm, args[0])
       
    if len(migrations) == 0:
        print('Bag file does not need any migrations.')
        exit(0)
        
    print('The following migrations need to occur:')
    all_rules = []
    for m in migrations:
        all_rules.extend(m[1])

        print_trans(m[0][0].old_class, m[0][-1].new_class, 0)
        if len(m[1]) > 0:
            print("    %d rules missing:" % len(m[1]))
            for r in m[1]:
                print_trans(r.old_class, r.new_class, 1)

    if options.rulefile is None:
        if all_rules == []:
            print("\nAll rules defined.  Bag is ready to be migrated")
        else:
            print("\nTo generate rules, please run with -g <rulefile>")
        exit(0)

    output = ''
    rules_left = mm.filter_rules_unique(all_rules)

    if rules_left == []:
        print("\nNo additional rule files needed to be generated.  %s not created."%(options.rulefile))
        exit(0)

    while len(rules_left) > 0:
        extra_rules = []
        for r in rules_left:
            if r.new_class is None:
                print('The message type %s appears to have moved.  Please enter the type to migrate it to.' % r.old_class._type)
                new_type = raw_input('>')
                new_class = roslib.message.get_message_class(new_type)
                while new_class is None:
                    print("\'%s\' could not be found in your system.  Please make sure it is built." % new_type)
                    new_type = raw_input('>')
                    new_class = roslib.message.get_message_class(new_type)
                new_rule = mm.make_update_rule(r.old_class, new_class)
                R = new_rule(mm, 'GENERATED.' + new_rule.__name__)
                R.find_sub_paths()
                new_rules = [r for r in mm.expand_rules(R.sub_rules) if r.valid == False]
                extra_rules.extend(new_rules)
                print('Creating the migration rule for %s requires additional missing rules:' % new_type)
                for nr in new_rules:
                    print_trans(nr.old_class, nr.new_class,1)
                output += R.get_class_def()
            else:
                output += r.get_class_def()
        rules_left = mm.filter_rules_unique(extra_rules)
    f = open(options.rulefile, 'a')
    f.write(output)
    f.close()

    print('\nThe necessary rule files have been written to: %s' % options.rulefile)

def compress_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag compress [options] BAGFILE1 [BAGFILE2 ...]',
                                   description='Compress one or more bag files.')
    parser.add_option(      '--output-dir', action='store',       dest='output_dir',  help='write to directory DIR', metavar='DIR')
    parser.add_option('-f', '--force',      action='store_true',  dest='force',       help='force overwriting of backup file if it exists')
    parser.add_option('-q', '--quiet',      action='store_true',  dest='quiet',       help='suppress noncritical messages')
    parser.add_option('-j', '--bz2',        action='store_const', dest='compression', help='use BZ2 compression', const=Compression.BZ2, default=Compression.BZ2)
    parser.add_option(      '--lz4',        action='store_const', dest='compression', help='use lz4 compression', const=Compression.LZ4)
    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must specify at least one bag file.')

    op = lambda inbag, outbag, quiet: change_compression_op(inbag, outbag, options.compression, options.quiet)

    bag_op(args, False, True, lambda b: False, op, options.output_dir, options.force, options.quiet)

def decompress_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag decompress [options] BAGFILE1 [BAGFILE2 ...]',
                                   description='Decompress one or more bag files.')
    parser.add_option(      '--output-dir', action='store',      dest='output_dir', help='write to directory DIR', metavar='DIR')
    parser.add_option('-f', '--force',      action='store_true', dest='force',      help='force overwriting of backup file if it exists')
    parser.add_option('-q', '--quiet',      action='store_true', dest='quiet',      help='suppress noncritical messages')

    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must specify at least one bag file.')
    
    op = lambda inbag, outbag, quiet: change_compression_op(inbag, outbag, Compression.NONE, options.quiet)
    
    bag_op(args, False, True, lambda b: False, op, options.output_dir, options.force, options.quiet)

def reindex_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag reindex [options] BAGFILE1 [BAGFILE2 ...]',
                                   description='Reindexes one or more bag files.')
    parser.add_option(      '--output-dir', action='store',      dest='output_dir', help='write to directory DIR', metavar='DIR')
    parser.add_option('-f', '--force',      action='store_true', dest='force',      help='force overwriting of backup file if it exists')
    parser.add_option('-q', '--quiet',      action='store_true', dest='quiet',      help='suppress noncritical messages')

    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must specify at least one bag file.')
    
    op = lambda inbag, outbag, quiet: reindex_op(inbag, outbag, options.quiet)

    bag_op(args, True, True, lambda b: b.version > 102, op, options.output_dir, options.force, options.quiet)

def encrypt_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag encrypt [options] BAGFILE1 [BAGFILE2 ...]',
                                   description='Encrypt one or more bag files.')
    parser.add_option(      '--output-dir', action='store',       dest='output_dir',  help='write to directory DIR', metavar='DIR')
    parser.add_option('-f', '--force',      action='store_true',  dest='force',       help='force overwriting of backup file if it exists')
    parser.add_option('-q', '--quiet',      action='store_true',  dest='quiet',       help='suppress noncritical messages')
    parser.add_option("-p", "--plugin",     action='store',       dest="plugin",      default='rosbag/AesCbcEncryptor', help='encryptor plugin name')
    parser.add_option("-r", "--param",      action='store',       dest="param",       default='*', help='encryptor plugin parameter')
    parser.add_option('-j', '--bz2',        action='store_const', dest='compression', help='use BZ2 compression', const=Compression.BZ2, default=Compression.NONE)
    parser.add_option(      '--lz4',        action='store_const', dest='compression', help='use lz4 compression', const=Compression.LZ4)
    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must specify at least one bag file.')

    op = lambda inbag, outbag, quiet: change_encryption_op(inbag, outbag, options.plugin, options.param, options.compression, options.quiet)

    bag_op(args, False, True, lambda b: False, op, options.output_dir, options.force, options.quiet)

def decrypt_cmd(argv):
    parser = optparse.OptionParser(usage='rosbag decrypt [options] BAGFILE1 [BAGFILE2 ...]',
                                   description='Decrypt one or more bag files.')
    parser.add_option(      '--output-dir', action='store',       dest='output_dir',  help='write to directory DIR', metavar='DIR')
    parser.add_option('-f', '--force',      action='store_true',  dest='force',       help='force overwriting of backup file if it exists')
    parser.add_option('-q', '--quiet',      action='store_true',  dest='quiet',       help='suppress noncritical messages')
    parser.add_option('-j', '--bz2',        action='store_const', dest='compression', help='use BZ2 compression', const=Compression.BZ2, default=Compression.NONE)
    parser.add_option(      '--lz4',        action='store_const', dest='compression', help='use lz4 compression', const=Compression.LZ4)
    (options, args) = parser.parse_args(argv)

    if len(args) < 1:
        parser.error('You must specify at least one bag file.')

    op = lambda inbag, outbag, quiet: change_encryption_op(inbag, outbag, 'rosbag/NoEncryptor', '*', options.compression, options.quiet)
    # Note the second paramater is True: Python Bag class cannot read index information from encrypted bag files
    bag_op(args, True, False, lambda b: False, op, options.output_dir, options.force, options.quiet)

def bag_op(inbag_filenames, allow_unindexed, open_inbag, copy_fn, op, output_dir=None, force=False, quiet=False):
    for inbag_filename in inbag_filenames:
        if open_inbag:
            # Check we can read the file
            try:
                inbag = Bag(inbag_filename, 'r', allow_unindexed=allow_unindexed)
            except ROSBagUnindexedException:
                print('ERROR bag unindexed: %s.  Run rosbag reindex.' % inbag_filename, file=sys.stderr)
                continue
            except (ROSBagException, IOError) as ex:
                print('ERROR reading %s: %s' % (inbag_filename, str(ex)), file=sys.stderr)
                continue

            # Determine whether we should copy the bag
            copy = copy_fn(inbag)

            inbag.close()
        else:
            copy = False

        # Determine filename for output bag
        if output_dir is None:
            outbag_filename = inbag_filename
        else:
            outbag_filename = os.path.join(output_dir, os.path.split(inbag_filename)[1])

        backup_filename = None
        if outbag_filename == inbag_filename:
            # Rename the input bag to ###.orig.###, and open for reading
            backup_filename = '%s.orig%s' % os.path.splitext(inbag_filename)
            
            if not force and os.path.exists(backup_filename):
                if not quiet:
                    print('Skipping %s. Backup path %s already exists.' % (inbag_filename, backup_filename), file=sys.stderr)
                continue
            
            try:
                if copy:
                    shutil.copy(inbag_filename, backup_filename)
                else:
                    os.rename(inbag_filename, backup_filename)
            except OSError as ex:
                print('ERROR %s %s to %s: %s' % ('copying' if copy else 'moving', inbag_filename, backup_filename, str(ex)), file=sys.stderr)
                continue
            
            source_filename = backup_filename
        else:
            if copy:
                shutil.copy(inbag_filename, outbag_filename)
                source_filename = outbag_filename
            else:
                source_filename = inbag_filename

        try:
            if open_inbag:
                inbag = Bag(source_filename, 'r', allow_unindexed=allow_unindexed)

                # Open the output bag file for writing
                try:
                    if copy:
                        outbag = Bag(outbag_filename, 'a', allow_unindexed=allow_unindexed)
                    else:
                        outbag = Bag(outbag_filename, 'w')
                except (ROSBagException, IOError) as ex:
                    print('ERROR writing to %s: %s' % (outbag_filename, str(ex)), file=sys.stderr)
                    inbag.close()
                    continue

                # Perform the operation
                try:
                    op(inbag, outbag, quiet=quiet)
                except ROSBagException as ex:
                    print('\nERROR operating on %s: %s' % (source_filename, str(ex)), file=sys.stderr)
                    inbag.close()
                    outbag.close()
                    continue

                outbag.close()
                inbag.close()
            else:
                # Open the output bag file for writing
                try:
                    if copy:
                        outbag = Bag(outbag_filename, 'a', allow_unindexed=allow_unindexed)
                    else:
                        outbag = Bag(outbag_filename, 'w')
                except (ROSBagException, IOError) as ex:
                    print('ERROR writing to %s: %s' % (outbag_filename, str(ex)), file=sys.stderr)
                    continue

                # Perform the operation
                try:
                    op(source_filename, outbag, quiet=quiet)
                except ROSBagException as ex:
                    print('\nERROR operating on %s: %s' % (source_filename, str(ex)), file=sys.stderr)
                    outbag.close()
                    continue

                outbag.close()

        except KeyboardInterrupt:
            if backup_filename is not None:
                try:
                    if copy:
                        os.remove(backup_filename)
                    else:
                        os.rename(backup_filename, inbag_filename)
                except OSError as ex:
                    print('ERROR %s %s to %s: %s', ('removing' if copy else 'moving', backup_filename, inbag_filename, str(ex)), file=sys.stderr)
                    break
    
        except (ROSBagException, IOError) as ex:
            print('ERROR operating on %s: %s' % (inbag_filename, str(ex)), file=sys.stderr)

def change_compression_op(inbag, outbag, compression, quiet):
    outbag.compression = compression

    if quiet:
        for topic, msg, t, conn_header in inbag.read_messages(raw=True, return_connection_header=True):
            outbag.write(topic, msg, t, raw=True, connection_header=conn_header)
    else:
        meter = ProgressMeter(outbag.filename, inbag._uncompressed_size)

        total_bytes = 0
        for topic, msg, t, conn_header in inbag.read_messages(raw=True, return_connection_header=True):
            msg_type, serialized_bytes, md5sum, pos, pytype = msg
    
            outbag.write(topic, msg, t, raw=True, connection_header=conn_header)
            
            total_bytes += len(serialized_bytes) 
            meter.step(total_bytes)
        
        meter.finish()

def reindex_op(inbag, outbag, quiet):
    if inbag.version == 102:
        if quiet:
            try:
                for offset in inbag.reindex():
                    pass
            except:
                pass

            for (topic, msg, t, conn_header) in inbag.read_messages(return_connection_header=True):
                outbag.write(topic, msg, t, connection_header=conn_header)
        else:
            meter = ProgressMeter(outbag.filename, inbag.size)
            try:
                for offset in inbag.reindex():
                    meter.step(offset)
            except:
                pass
            meter.finish()

            meter = ProgressMeter(outbag.filename, inbag.size)
            for (topic, msg, t, conn_header) in inbag.read_messages(return_connection_header=True):
                outbag.write(topic, msg, t, connection_header=conn_header)
                meter.step(inbag._file.tell())
            meter.finish()
    else:
        if quiet:
            try:
                for offset in outbag.reindex():
                    pass
            except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
                raise
            except:
                pass
        else:
            meter = ProgressMeter(outbag.filename, outbag.size)
            try:
                for offset in outbag.reindex():
                    meter.step(offset)
            except (ROSBagEncryptNotSupportedException, ROSBagEncryptException) as ex:
                raise
            except:
                pass
            meter.finish()

def change_encryption_op(inbag, outbag, plugin, param, compression, quiet):
    # Output file must be closed before written by the encrypt process
    outbag.close()

    encryptpath = roslib.packages.find_node('rosbag', 'encrypt')
    if not encryptpath:
        parser.error("Cannot find rosbag/encrypt executable")
    cmd = [encryptpath[0]]
    if type(inbag) is str:
        cmd.extend([inbag])
    else:
        cmd.extend([inbag.filename])
    cmd.extend(['-o', outbag.filename])
    cmd.extend(['-p', plugin])
    cmd.extend(['-r', param])
    if compression == 'bz2':
        cmd.extend(['-j'])
    elif compression == 'lz4':
        cmd.extend(['--lz4'])
    if quiet:
        cmd.extend(['-q'])

    old_handler = signal.signal(
        signal.SIGTERM,
        lambda signum, frame: _stop_process(signum, frame, old_handler, process)
    )

    process = subprocess.Popen(cmd)
    process.wait()

class RosbagCmds(UserDict):
    def __init__(self):
        UserDict.__init__(self)
        self._description = {}
        self['help'] = self.help_cmd

    def add_cmd(self, name, function, description):
        self[name] = function
        self._description[name] = description
        
    def get_valid_cmds(self):
        str = "Available subcommands:\n"
        for k in sorted(self.keys()):
            str += "   %s  " % k
            if k in self._description.keys():
                str +="\t%s" % self._description[k]
            str += "\n"
        return str

    def help_cmd(self,argv):
        argv = [a for a in argv if a != '-h' and a != '--help']

        if len(argv) == 0:
            print('Usage: rosbag <subcommand> [options] [args]')
            print()
            print("A bag is a file format in ROS for storing ROS message data. The rosbag command can record, replay and manipulate bags.")
            print()
            print(self.get_valid_cmds())
            print('For additional information, see http://wiki.ros.org/rosbag')
            print()
            return

        cmd = argv[0]
        if cmd in self:
            self[cmd](['-h'])
        else:
            print("Unknown command: '%s'" % cmd, file=sys.stderr)
            print(self.get_valid_cmds(), file=sys.stderr)

class ProgressMeter(object):
    def __init__(self, path, bytes_total, refresh_rate=1.0):
        self.path           = path
        self.bytes_total    = bytes_total
        self.refresh_rate   = refresh_rate
        
        self.elapsed        = 0.0
        self.update_elapsed = 0.0
        self.bytes_read     = 0.0

        self.start_time     = time.time()

        self._update_progress()

    def step(self, bytes_read, force_update=False):
        self.bytes_read = bytes_read
        self.elapsed    = time.time() - self.start_time
        
        if force_update or self.elapsed - self.update_elapsed > self.refresh_rate:
            self._update_progress()
            self.update_elapsed = self.elapsed

    def _update_progress(self):
        max_path_len = self.terminal_width() - 37
        path = self.path
        if len(path) > max_path_len:
            path = '...' + self.path[-max_path_len + 3:]

        bytes_read_str  = self._human_readable_size(float(self.bytes_read))
        bytes_total_str = self._human_readable_size(float(self.bytes_total))
        
        if self.bytes_read < self.bytes_total:
            complete_fraction = float(self.bytes_read) / self.bytes_total
            pct_complete      = int(100.0 * complete_fraction)

            if complete_fraction > 0.0:
                eta = self.elapsed * (1.0 / complete_fraction - 1.0)
                eta_min, eta_sec = eta / 60, eta % 60
                if eta_min > 99:
                    eta_str = '--:--'
                else:
                    eta_str = '%02d:%02d' % (eta_min, eta_sec)
            else:
                eta_str = '--:--'

            progress = '%-*s %3d%% %8s / %8s %s ETA' % (max_path_len, path, pct_complete, bytes_read_str, bytes_total_str, eta_str)
        else:
            progress = '%-*s 100%% %19s %02d:%02d    ' % (max_path_len, path, bytes_total_str, self.elapsed / 60, self.elapsed % 60)

        print('\r', progress, end='')
        sys.stdout.flush()
        
    def _human_readable_size(self, size):
        multiple = 1024.0
        for suffix in ['KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB']:
            size /= multiple
            if size < multiple:
                return '%.1f %s' % (size, suffix)
    
        raise ValueError('number too large')

    def finish(self):
        self.step(self.bytes_total, force_update=True)
        print()

    @staticmethod
    def terminal_width():
        """Estimate the width of the terminal"""
        width = 0
        try:
            import struct, fcntl, termios
            s     = struct.pack('HHHH', 0, 0, 0, 0)
            x     = fcntl.ioctl(1, termios.TIOCGWINSZ, s)
            width = struct.unpack('HHHH', x)[1]
        except (IOError, ImportError):
            pass
        if width <= 0:
            try:
                width = int(os.environ['COLUMNS'])
            except:
                pass
        if width <= 0:
            width = 80
    
        return width

def rosbagmain(argv=None):
    cmds = RosbagCmds()
    cmds.add_cmd('record', record_cmd, "Record a bag file with the contents of specified topics.")
    cmds.add_cmd('info', info_cmd, 'Summarize the contents of one or more bag files.')
    cmds.add_cmd('play', play_cmd, "Play back the contents of one or more bag files in a time-synchronized fashion.")
    cmds.add_cmd('check', check_cmd, 'Determine whether a bag is playable in the current system, or if it can be migrated.')
    cmds.add_cmd('fix', fix_cmd, 'Repair the messages in a bag file so that it can be played in the current system.')
    cmds.add_cmd('filter', filter_cmd, 'Filter the contents of the bag.')
    cmds.add_cmd('compress', compress_cmd, 'Compress one or more bag files.')
    cmds.add_cmd('decompress', decompress_cmd, 'Decompress one or more bag files.')
    cmds.add_cmd('reindex', reindex_cmd, 'Reindexes one or more bag files.')
    if sys.platform != 'win32':
        cmds.add_cmd('encrypt', encrypt_cmd, 'Encrypt one or more bag files.')
        cmds.add_cmd('decrypt', decrypt_cmd, 'Decrypt one or more bag files.')

    if argv is None:
        argv = sys.argv

    if '-h' in argv or '--help' in argv:
        argv = [a for a in argv if a != '-h' and a != '--help']
        argv.insert(1, 'help')

    if len(argv) > 1:
        cmd = argv[1]
    else:
        cmd = 'help'

    try:
        if cmd in cmds:
            cmds[cmd](argv[2:])
        else:
            cmds['help']([cmd])
    except KeyboardInterrupt:
        pass
