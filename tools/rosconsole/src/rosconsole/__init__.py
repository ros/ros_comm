#!/usr/bin/env python

import os
import sys
import socket

import rospy
import rosgraph

from .logger_level_service_caller import LoggerLevelServiceCaller
from .logger_level_service_caller import ROSConsoleException

NAME = 'rosconsole'


def error(status, msg):
    sys.stderr.write("%s: error: %s\n" % (NAME, msg))
    sys.exit(status)


def _get_cmd_list_optparse(args):
    from optparse import OptionParser

    usage = "usage: %prog list <node>\n"
    parser = OptionParser(usage=usage, prog=NAME)

    return parser


def _rosconsole_cmd_list(argv):

    args = argv[2:]
    parser = _get_cmd_list_optparse(args)
    (options, args) = parser.parse_args(args)

    if not args:
        parser.error("you must specify a node to list loggers")
    elif len(args) > 1:
        parser.error("you may only specify one node to list")

    logger_level = LoggerLevelServiceCaller()

    loggers = logger_level.get_loggers(args[0])

    output = '\n'.join(loggers)
    print(output)


def _get_cmd_set_optparse(args):
    from optparse import OptionParser

    usage = "usage: %prog set <node> <logger> <level>\n"
    levels = ', '.join(LoggerLevelServiceCaller().get_levels())
    usage += "\n <level> must be one of [" + levels + "]"
    parser = OptionParser(usage=usage, prog=NAME)

    return parser


def _rosconsole_cmd_set(argv):

    args = argv[2:]
    parser = _get_cmd_set_optparse(args)
    (options, args) = parser.parse_args(args)

    if len(args) < 3:
        parser.error("you must specify a node, a logger and a level")

    logger_level = LoggerLevelServiceCaller()

    loggers = logger_level.get_loggers(args[0])

    if args[1] not in logger_level._current_levels:
        error(2, "node " + args[0] + " does not contain logger " + args[1])

    # Match case-insensitively
    if args[2].lower() not in logger_level.get_levels():
        parser.error("invalid level")

    logger_level.send_logger_change_message(args[0], args[1], args[2])


def _get_cmd_get_optparse(args):
    from optparse import OptionParser

    usage = "usage: %prog get <node> <logger>\n"
    parser = OptionParser(usage=usage, prog=NAME)

    return parser


def _rosconsole_cmd_get(argv):

    args = argv[2:]
    parser = _get_cmd_get_optparse(args)
    (options, args) = parser.parse_args(args)

    if len(args) < 2:
        parser.error("you must specify a node and a logger")

    logger_level = LoggerLevelServiceCaller()

    loggers = logger_level.get_loggers(args[0])

    if args[1] not in logger_level._current_levels:
        error(2, "node " + args[0] + " does not contain logger " + args[1])

    print(logger_level._current_levels[args[1]])


def _fullusage():
    print("""rosconsole is a command-line tool for configuring the logger level of ROS nodes.

Commands:
\trosconsole get\tdisplay level for a logger
\trosconsole list\tlist loggers for a node
\trosconsole set\tset level for a logger

Type rosconsole <command> -h for more detailed usage, e.g. 'rosconsole list -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))


def main(argv=None):
    if argv is None:
        argv = sys.argv

    # Initialize ourselves as a node, to ensure handling of namespace and
    # remapping arguments
    rospy.init_node('rosconsole', anonymous=True)
    argv = rospy.myargv(argv)

    # process argv
    if len(argv) == 1:
        _fullusage()

    try:
        command = argv[1]
        if command == 'get':
            _rosconsole_cmd_get(argv)
        elif command == 'list':
            _rosconsole_cmd_list(argv)
        elif command == 'set':
            _rosconsole_cmd_set(argv)
        else:
            _fullusage()
    except socket.error as e:
        error(1, "Network communication failed. Most likely failed to communicate with master.\n")
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        error(1, str(e))
    except ROSConsoleException as e:
        error(1, str(e))
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
