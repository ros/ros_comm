/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/transport/transport.h"
#include "ros/console.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>

#ifndef NI_MAXHOST
  #define NI_MAXHOST 1025
#endif

namespace ros
{

Transport::Transport()
: only_localhost_allowed_(false)
{
  char *ros_ip_env = NULL, *ros_hostname_env = NULL;
  #ifdef _MSC_VER
    _dupenv_s(&ros_ip_env, NULL, "ROS_IP");
    _dupenv_s(&ros_hostname_env, NULL, "ROS_HOSTNAME");
  #else
    ros_ip_env = getenv("ROS_IP");
    ros_hostname_env = getenv("ROS_HOSTNAME");
  #endif
  if (ros_hostname_env && !strcmp(ros_hostname_env, "localhost"))
    only_localhost_allowed_ = true;
  else if (ros_ip_env && !strcmp(ros_ip_env, "localhost"))
    only_localhost_allowed_ = true;
  else if (ros_ip_env && !strncmp(ros_ip_env, "127.", 4))
    only_localhost_allowed_ = true;

  char our_hostname[256] = {0};
  gethostname(our_hostname, sizeof(our_hostname)-1);
  allowed_hosts_.push_back(std::string(our_hostname));
  allowed_hosts_.push_back("localhost");
  // for ipv4 loopback, we'll explicitly search for 127.* in isHostAllowed()
  // now we need to iterate all local interfaces and add their addresses
  // from the getifaddrs manpage:  (maybe something similar for windows ?) 
  ifaddrs *ifaddr;
  if (-1 == getifaddrs(&ifaddr))
  {
    ROS_ERROR("getifaddr() failed");
    return;
  }
  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next)
  {
    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET && family != AF_INET6)
      continue; // we're only looking for IP addresses
    char addr[NI_MAXHOST] = {0};
    if (getnameinfo(ifa->ifa_addr,
                    (family == AF_INET) ? sizeof(sockaddr_in)
                                        : sizeof(sockaddr_in6),
                    addr, NI_MAXHOST,
                    NULL, 0, NI_NUMERICHOST))
    {
      ROS_ERROR("getnameinfo() failed");
      continue;
    }
    allowed_hosts_.push_back(std::string(addr));
  }
}

bool Transport::isHostAllowed(const std::string &host) const
{
  if (!only_localhost_allowed_)
    return true; // doesn't matter; we'll connect to anybody

  if (host.length() >= 4 && host.substr(0, 4) == std::string("127."))
    return true; // ipv4 localhost
  // now, loop through the list of valid hostnames and see if we find it
  for (std::vector<std::string>::iterator it = allowed_hosts_.begin(); 
       it != allowed_hosts_.end(); ++it)
  {
    if (host == *it)
      return true; // hooray
  }
  ROS_WARN("ROS_HOSTNAME / ROS_IP is set to only allow local connections, so "
           "a requested connection to '%s' is being rejected.", host.c_str());
  return false; // sadness
}

}

