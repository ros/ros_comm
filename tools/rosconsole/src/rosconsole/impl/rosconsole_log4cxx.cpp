/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

// Author: Josh Faust

#if defined(__APPLE__) && defined(__GNUC__) && defined(__llvm__) && !defined(__clang__) && (__GNUC__ == 4) && (__GNUC_MINOR__ == 2)
#error This code is known to provoke a compiler crash with llvm-gcc 4.2. You will have better luck with clang++. See code.ros.org/trac/ros/ticket/3626
#endif

#include "ros/console.h"
#include "ros/assert.h"
#include <ros/time.h>
#include "log4cxx/appenderskeleton.h"
#include "log4cxx/spi/loggingevent.h"
#include "log4cxx/level.h"
#include "log4cxx/propertyconfigurator.h"
#ifdef _MSC_VER
  // Have to be able to encode wchar LogStrings on windows.
  #include "log4cxx/helpers/transcoder.h"
#endif

#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <boost/regex.hpp>

#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <cstring>
#include <stdexcept>

namespace ros
{
namespace console
{
namespace impl
{

log4cxx::LevelPtr g_level_lookup[ levels::Count ] =
{
  log4cxx::Level::getDebug(),
  log4cxx::Level::getInfo(),
  log4cxx::Level::getWarn(),
  log4cxx::Level::getError(),
  log4cxx::Level::getFatal(),
};


class ROSConsoleStdioAppender : public log4cxx::AppenderSkeleton
{
public:
  ~ROSConsoleStdioAppender()
  {
  }

protected:
  virtual void append(const log4cxx::spi::LoggingEventPtr& event, 
                      log4cxx::helpers::Pool&)
  {
    levels::Level level = levels::Count;
    if (event->getLevel() == log4cxx::Level::getDebug())
    {
      level = levels::Debug;
    }
    else if (event->getLevel() == log4cxx::Level::getInfo())
    {
      level = levels::Info;
    }
    else if (event->getLevel() == log4cxx::Level::getWarn())
    {
      level = levels::Warn;
    }
    else if (event->getLevel() == log4cxx::Level::getError())
    {
      level = levels::Error;
    }
    else if (event->getLevel() == log4cxx::Level::getFatal())
    {
      level = levels::Fatal;
    }
#ifdef _MSC_VER
    LOG4CXX_ENCODE_CHAR(tmpstr, event->getMessage());  // has to handle LogString with wchar types.
    std::string msg = tmpstr  // tmpstr gets instantiated inside the LOG4CXX_ENCODE_CHAR macro
#else
    std::string msg = event->getMessage();
#endif
    const log4cxx::spi::LocationInfo& location_info = event->getLocationInformation();
    ::ros::console::backend::print(this, level, msg.c_str(), location_info.getFileName(), location_info.getMethodName().c_str(), location_info.getLineNumber());
  }

  virtual void close()
  {
  }
  virtual bool requiresLayout() const
  {
    return false;
  }
};

void initialize()
{
  // First set up some sane defaults programmatically.
  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
  ros_logger->setLevel(log4cxx::Level::getInfo());

  log4cxx::LoggerPtr roscpp_superdebug = log4cxx::Logger::getLogger("ros.roscpp.superdebug");
  roscpp_superdebug->setLevel(log4cxx::Level::getWarn());

  // Next try to load the default config file from ROS_ROOT/config/rosconsole.config
  char* ros_root_cstr = NULL;
#ifdef _MSC_VER
  _dupenv_s(&ros_root_cstr, NULL, "ROS_ROOT");
#else
  ros_root_cstr = getenv("ROS_ROOT");
#endif
  if (ros_root_cstr)
  {
    std::string config_file = std::string(ros_root_cstr) + "/config/rosconsole.config";
    FILE* config_file_ptr = fopen( config_file.c_str(), "r" );
    if( config_file_ptr ) // only load it if the file exists, to avoid a warning message getting printed.
    {
      fclose( config_file_ptr );
      log4cxx::PropertyConfigurator::configure(config_file);
    }
  }
  char* config_file_cstr = NULL;
#ifdef _MSC_VER
  _dupenv_s(&config_file_cstr, NULL, "ROSCONSOLE_CONFIG_FILE");
#else
  config_file_cstr = getenv("ROSCONSOLE_CONFIG_FILE");
#endif
  if ( config_file_cstr )
  {
    std::string config_file = config_file_cstr;
    log4cxx::PropertyConfigurator::configure(config_file);
  }

  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
  logger->addAppender(new ROSConsoleStdioAppender);
#ifdef _MSC_VER
  if ( ros_root_cstr != NULL ) {
	  free(ros_root_cstr);
  }
  if ( config_file_cstr != NULL ) {
	  free(config_file_cstr);
  }
  if ( format_string != NULL ) {
	  free(format_string);
  }
  // getenv implementations don't need free'ing.
#endif
}


void print(void* handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  log4cxx::Logger* logger  = (log4cxx::Logger*)handle;
  try
  {
    logger->forcedLog(g_level_lookup[level], str, log4cxx::spi::LocationInfo(file, function, line));
  }
  catch (std::exception& e)
  {
    fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
  }
}

bool isEnabledFor(void* handle, ::ros::console::Level level)
{
  log4cxx::Logger* logger  = (log4cxx::Logger*)handle;
  return logger->isEnabledFor(g_level_lookup[level]);
}

void* getHandle(const std::string& name)
{
  return log4cxx::Logger::getLogger(name);
}

std::string getName(void* handle)
{
  log4cxx::Logger* logger  = (log4cxx::Logger*)handle;
#ifdef _MSC_VER
  LOG4CXX_ENCODE_CHAR(tmpstr, event->getName());  // has to handle LogString with wchar types.
  return tmpstr  // tmpstr gets instantiated inside the LOG4CXX_ENCODE_CHAR macro
#else
  return logger->getName();
#endif
}

bool get_loggers(std::map<std::string, levels::Level>& loggers)
{
  log4cxx::spi::LoggerRepositoryPtr repo = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME)->getLoggerRepository();

  log4cxx::LoggerList current_loggers = repo->getCurrentLoggers();
  log4cxx::LoggerList::iterator it = current_loggers.begin();
  log4cxx::LoggerList::iterator end = current_loggers.end();
  for (; it != end; ++it)
  {
    std::string name;
    #ifdef _MSC_VER
      LOG4CXX_ENCODE_CHAR(name, (*it)->getName()); // has to handle LogString with wchar types.
    #else
      name = (*it)->getName();
    #endif

    const log4cxx::LevelPtr& log4cxx_level = (*it)->getEffectiveLevel();
    levels::Level level;
    if (log4cxx_level == log4cxx::Level::getDebug())
    {
      level = levels::Debug;
    }
    else if (log4cxx_level == log4cxx::Level::getInfo())
    {
      level = levels::Info;
    }
    else if (log4cxx_level == log4cxx::Level::getWarn())
    {
      level = levels::Warn;
    }
    else if (log4cxx_level == log4cxx::Level::getError())
    {
      level = levels::Error;
    }
    else if (log4cxx_level == log4cxx::Level::getFatal())
    {
      level = levels::Fatal;
    }
    loggers[name] = level;
  }

  return true;
}

bool set_logger_level(const std::string& name, levels::Level level)
{
  log4cxx::LevelPtr log4cxx_level;
  if (level == levels::Debug)
  {
    log4cxx_level = log4cxx::Level::getDebug();
  }
  else if (level == levels::Info)
  {
    log4cxx_level = log4cxx::Level::getInfo();
  }
  else if (level == levels::Warn)
  {
    log4cxx_level = log4cxx::Level::getWarn();
  }
  else if (level == levels::Error)
  {
    log4cxx_level = log4cxx::Level::getError();
  }
  else if (level == levels::Fatal)
  {
    log4cxx_level = log4cxx::Level::getFatal();
  }
  else
  {
    return false;
  }

  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(name);
  logger->setLevel(log4cxx_level);
  ::ros::console::backend::notifyLoggerLevelsChanged();
  return true;
}

class Log4cxxAppender : public log4cxx::AppenderSkeleton
{
public:
  Log4cxxAppender(ros::console::LogAppender* appender) : appender_(appender) {}
  ~Log4cxxAppender() {}

protected:
  virtual void append(const log4cxx::spi::LoggingEventPtr& event, log4cxx::helpers::Pool& pool)
  {
    levels::Level level;
    if (event->getLevel() == log4cxx::Level::getFatal())
    {
      level = levels::Fatal;
    }
    else if (event->getLevel() == log4cxx::Level::getError())
    {
      level = levels::Error;
    }
    else if (event->getLevel() == log4cxx::Level::getWarn())
    {
      level = levels::Warn;
    }
    else if (event->getLevel() == log4cxx::Level::getInfo())
    {
      level = levels::Info;
    }
    else if (event->getLevel() == log4cxx::Level::getDebug())
    {
      level = levels::Debug;
    }
    else
    {
      return;
    }

    #ifdef _MSC_VER
      LOG4CXX_ENCODE_CHAR(tmpstr, event->getMessage());  // has to handle LogString with wchar types.
      std::string msg = tmpstr  // tmpstr gets instantiated inside the LOG4CXX_ENCODE_CHAR macro
    #else
      std::string msg = event->getMessage();
    #endif

    const log4cxx::spi::LocationInfo& info = event->getLocationInformation();
    appender_->log(level, msg.c_str(), info.getFileName(), info.getMethodName().c_str(), info.getLineNumber());
  }

  virtual void close() {}
  virtual bool requiresLayout() const { return false; }
  ros::console::LogAppender* appender_;
};

Log4cxxAppender* g_log4cxx_appender;

void register_appender(LogAppender* appender)
{
  g_log4cxx_appender = new Log4cxxAppender(appender);
  const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
  logger->addAppender(g_log4cxx_appender);
}

void shutdown()
{
  const log4cxx::LoggerPtr& logger = log4cxx::Logger::getLogger(ROSCONSOLE_ROOT_LOGGER_NAME);
  logger->removeAppender(g_log4cxx_appender);
  g_log4cxx_appender = 0;
  // reset this so that the logger doesn't get crashily destroyed
  // again during global destruction.  
  //
  // See https://code.ros.org/trac/ros/ticket/3271
  //
  log4cxx::Logger::getRootLogger()->getLoggerRepository()->shutdown();
}

} // namespace impl
} // namespace console
} // namespace ros
