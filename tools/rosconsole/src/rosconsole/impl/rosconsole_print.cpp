/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/console.h"

namespace ros
{
namespace console
{
namespace impl
{

LogAppender* rosconsole_print_appender = 0;

void initialize()
{}

void print(void* handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  g_formatter.print(0, level, str, file, function, line);
  if(rosconsole_print_appender)
  {
    rosconsole_print_appender->log(level, str, file, function, line);
  }
}

bool isEnabledFor(void* handle, ::ros::console::Level level)
{
  return level != ::ros::console::levels::Debug;
}

void* getHandle(const std::string& name)
{
  return 0;
}

std::string getName(void* handle)
{
  return "";
}

void register_appender(LogAppender* appender)
{
  rosconsole_print_appender = appender;
}

void shutdown()
{}

bool get_loggers(std::map<std::string, levels::Level>& loggers)
{
  return true;
}

bool set_logger_level(const std::string& name, levels::Level level)
{
  return false;
}

} // namespace impl
} // namespace console
} // namespace ros
