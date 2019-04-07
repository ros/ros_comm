# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
# Revision $Id: environment.py 4428 2009-05-05 05:48:36Z jfaustwg $

import os
import socket
import stat
import string
import sys

import rosgraph
import rosgraph.network

from roswtf.rules import warning_rule, error_rule

# #1220
def ip_check(ctx):
    # best we can do is compare roslib's routine against socket resolution and make sure they agree
    local_addrs = rosgraph.network.get_local_addresses()

    if rosgraph.network.use_ipv6():
        resolved_ips = [host[4][0] for host in socket.getaddrinfo(socket.gethostname(), 0, 0, 0, socket.SOL_TCP)]
    else:
        resolved_ips = [host[4][0] for host in socket.getaddrinfo(socket.gethostname(), 0, socket.AF_INET, 0, socket.SOL_TCP)]

    global_ips = [ ip for ip in resolved_ips if not ip.startswith('127.') and not ip == '::1']

    remote_ips = list(set(global_ips) - set(local_addrs))
    if remote_ips:
        retval = "Local hostname [%s] resolves to [%s], which does not appear to be a local IP address %s." % (socket.gethostname(), ','.join(remote_ips), str(local_addrs))
        # IPv6 support % to denote zone/scope ids. The value is expanded
        # in other functions, this is why we are using replace command in
        # the return. For more info https://github.com/ros/ros_comm/pull/598
        return retval.replace('%', '%%')

# suggestion by mquigley based on laptop dhcp issues    
def ros_hostname_check(ctx):
    """Make sure that ROS_HOSTNAME resolves to a local IP address"""
    if not rosgraph.ROS_HOSTNAME in ctx.env:
        return

    hostname = ctx.env[rosgraph.ROS_HOSTNAME]
    try:
        resolved_ips = [host[4][0] for host in socket.getaddrinfo(hostname, 0, 0, 0, socket.SOL_TCP)]
    except socket.gaierror:
        return "ROS_HOSTNAME [%s] cannot be resolved to an IP address"%(hostname)
    
    # best we can do is compare roslib's routine against socket resolution and make sure they agree
    local_addrs = rosgraph.network.get_local_addresses()

    remote_ips = list(set(resolved_ips) - set(local_addrs))
    if remote_ips:
        return "ROS_HOSTNAME [%s] resolves to [%s], which does not appear to be a local IP address %s."%(hostname, ','.join(remote_ips), str(local_addrs))

def ros_ip_check(ctx):
    """Make sure that ROS_IP is a local IP address"""
    if not rosgraph.ROS_IP in ctx.env:
        return

    ip = ctx.env[rosgraph.ROS_IP]
    
    # best we can do is compare roslib's routine against socket resolution and make sure they agree
    addrs = rosgraph.network.get_local_addresses()

    if " " in ip:
        return "ROS_IP [%s] contains whitespace. This is not a valid IP."%ip

    if ip not in addrs:
        return "ROS_IP [%s] does not appear to be an IP address of a local network interface (one of %s)."%(ip, str(addrs))
    
# Error/Warning Rules

warnings = [
    (ros_hostname_check,
     "ROS_HOSTNAME may be incorrect: "),
    (ros_ip_check,
     "ROS_IP may be incorrect: "),

    ]

errors = [
    (ip_check,
     "Local network configuration is invalid: "),
    ]

def wtf_check(ctx):
    for r in warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in errors:
        error_rule(r, r[0](ctx), ctx)
        
