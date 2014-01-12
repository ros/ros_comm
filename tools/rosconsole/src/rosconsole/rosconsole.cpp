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

#include <boost/thread.hpp>
#include <boost/shared_array.hpp>
#include <boost/regex.hpp>

#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <cstring>
#include <stdexcept>

// declare interface for rosconsole implementations
namespace ros
{
namespace console
{
namespace impl
{

void initialize();

void shutdown();

void register_appender(LogAppender* appender);

void print(void* handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line);

bool isEnabledFor(void* handle, ::ros::console::Level level);

void* getHandle(const std::string& name);

std::string getName(void* handle);

bool get_loggers(std::map<std::string, levels::Level>& loggers);

bool set_logger_level(const std::string& name, levels::Level level);

} // namespace impl


bool g_initialized = false;
bool g_shutting_down = false;
boost::mutex g_init_mutex;

#ifdef ROSCONSOLE_BACKEND_LOG4CXX
log4cxx::LevelPtr g_level_lookup[levels::Count] =
{
  log4cxx::Level::getDebug(),
  log4cxx::Level::getInfo(),
  log4cxx::Level::getWarn(),
  log4cxx::Level::getError(),
  log4cxx::Level::getFatal(),
};
#endif
std::string g_last_error_message = "Unknown Error";

#ifdef WIN32
  #define COLOR_NORMAL ""
  #define COLOR_RED ""
  #define COLOR_GREEN ""
  #define COLOR_YELLOW ""
#else
  #define COLOR_NORMAL "\033[0m"
  #define COLOR_RED "\033[31m"
  #define COLOR_GREEN "\033[32m"
  #define COLOR_YELLOW "\033[33m"
#endif
const char* g_format_string = "[${severity}] [${time}]: ${message}";

typedef std::map<std::string, std::string> M_string;
M_string g_extra_fixed_tokens;

void setFixedFilterToken(const std::string& key, const std::string& val)
{
  g_extra_fixed_tokens[key] = val;
}

struct FixedToken : public Token
{
  FixedToken(const std::string& str)
  : str_(str)
  {}

  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char*, int)
  {
    return str_.c_str();
  }

  std::string str_;
};

struct FixedMapToken : public Token
{
  FixedMapToken(const std::string& str)
  : str_(str)
  {}

  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char*, int)
  {
    M_string::iterator it = g_extra_fixed_tokens.find(str_);
    if (it == g_extra_fixed_tokens.end())
    {
      return ("${" + str_ + "}").c_str();
    }

    return it->second.c_str();
  }

  std::string str_;
};

struct PlaceHolderToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char*, int)
  {
    return "PLACEHOLDER";
  }
};

struct SeverityToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
  {
    if (level == levels::Fatal)
    {
      return "FATAL";
    }
    else if (level == levels::Error)
    {
      return "ERROR";
    }
    else if (level == levels::Warn)
    {
      return " WARN";
    }
    else if (level == levels::Info)
    {
      return " INFO";
    }
    else if (level == levels::Debug)
    {
      return "DEBUG";
    }

    return "UNKNO";
  }
};

struct MessageToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char* str, const char*, const char*, int)
  {
    return str;
  }
};

struct TimeToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char*, int)
  {
    std::stringstream ss;
    if (ros::Time::isValid() && ros::Time::isSimTime())
    {
      ss << ros::WallTime::now() << ", " << ros::Time::now();
    }
    else
    {
      ss << ros::WallTime::now();
    }
    return ss.str();
  }
};

struct ThreadToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char*, int)
  {
    std::stringstream ss;
    ss << boost::this_thread::get_id();
    return ss.str();
  }
};

struct LoggerToken : public Token
{
  virtual std::string getString(void* logger_handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
  {
    return ::ros::console::impl::getName(logger_handle);
  }
};

struct FileToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char*, const char* file, const char*, int)
  {
    return file;
  }
};

struct FunctionToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char* function, int)
  {
    return function;
  }
};

struct LineToken : public Token
{
  virtual std::string getString(void*, ::ros::console::Level, const char*, const char*, const char*, int line)
  {
    std::stringstream ss;
    ss << line;
    return ss.str();
  }
};

TokenPtr createTokenFromType(const std::string& type)
{
  if (type == "severity")
  {
    return TokenPtr(new SeverityToken());
  }
  else if (type == "message")
  {
    return TokenPtr(new MessageToken());
  }
  else if (type == "time")
  {
    return TokenPtr(new TimeToken());
  }
  else if (type == "thread")
  {
    return TokenPtr(new ThreadToken());
  }
  else if (type == "logger")
  {
    return TokenPtr(new LoggerToken());
  }
  else if (type == "file")
  {
    return TokenPtr(new FileToken());
  }
  else if (type == "line")
  {
    return TokenPtr(new LineToken());
  }
  else if (type == "function")
  {
    return TokenPtr(new FunctionToken());
  }

  return TokenPtr(new FixedMapToken(type));
}

void Formatter::init(const char* fmt)
{
  format_ = fmt;

  boost::regex e("\\$\\{([a-z|A-Z]+)\\}");
  boost::match_results<std::string::const_iterator> results;
  std::string::const_iterator start, end;
  start = format_.begin();
  end = format_.end();
  bool matched_once = false;
  std::string last_suffix;
  while (boost::regex_search(start, end, results, e))
  {
#if 0
    for (size_t i = 0; i < results.size(); ++i)
    {
      std::cout << i << "|" << results.prefix() << "|" <<  results[i] << "|" << results.suffix() << std::endl;
    }
#endif

    std::string token = results[1];
    last_suffix = results.suffix();
    tokens_.push_back(TokenPtr(new FixedToken(results.prefix())));
    tokens_.push_back(createTokenFromType(token));

    start = results[0].second;
    matched_once = true;
  }

  if (matched_once)
  {
    tokens_.push_back(TokenPtr(new FixedToken(last_suffix)));
  }
  else
  {
    tokens_.push_back(TokenPtr(new FixedToken(format_)));
  }
}

void Formatter::print(void* logger_handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  const char* color = NULL;
  FILE* f = stdout;

  if (level == levels::Fatal)
  {
    color = COLOR_RED;
    f = stderr;
  }
  else if (level == levels::Error)
  {
    color = COLOR_RED;
    f = stderr;
  }
  else if (level == levels::Warn)
  {
    color = COLOR_YELLOW;
  }
  else if (level == levels::Info)
  {
    color = COLOR_NORMAL;
  }
  else if (level == levels::Debug)
  {
    color = COLOR_GREEN;
  }

  ROS_ASSERT(color != NULL);

  std::stringstream ss;
  ss << color;
  V_Token::iterator it = tokens_.begin();
  V_Token::iterator end = tokens_.end();
  for (; it != end; ++it)
  {
    ss << (*it)->getString(logger_handle, level, str, file, function, line);
  }
  ss << COLOR_NORMAL;

  fprintf(f, "%s\n", ss.str().c_str());
}

Formatter g_formatter;


void _print(void* logger_handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  g_formatter.print(logger_handle, level, str, file, function, line);
}

void initialize()
{
  boost::mutex::scoped_lock lock(g_init_mutex);

  if (!g_initialized)
  {
    // Check for the format string environment variable
    char* format_string = NULL;
#ifdef _MSC_VER
    _dupenv_s(&format_string, NULL, "ROSCONSOLE_FORMAT");
#else
    format_string =  getenv("ROSCONSOLE_FORMAT");
#endif
    if (format_string)
    {
      g_format_string = format_string;
    }

    g_formatter.init(g_format_string);
    backend::function_notifyLoggerLevelsChanged = notifyLoggerLevelsChanged;
    backend::function_print = _print;

    ::ros::console::impl::initialize();
    g_initialized = true;
  }
}

void vformatToBuffer(boost::shared_array<char>& buffer, size_t& buffer_size, const char* fmt, va_list args)
{
#ifdef _MSC_VER
  va_list arg_copy = args; // dangerous?
#else
  va_list arg_copy;
  va_copy(arg_copy, args);
#endif
#ifdef _MSC_VER
  size_t total = vsnprintf_s(buffer.get(), buffer_size, buffer_size, fmt, args);
#else
  size_t total = vsnprintf(buffer.get(), buffer_size, fmt, args);
#endif
  if (total >= buffer_size)
  {
    buffer_size = total + 1;
    buffer.reset(new char[buffer_size]);

#ifdef _MSC_VER
    vsnprintf_s(buffer.get(), buffer_size, buffer_size, fmt, arg_copy);
#else
    vsnprintf(buffer.get(), buffer_size, fmt, arg_copy);
#endif
  }
  va_end(arg_copy);
}

void formatToBuffer(boost::shared_array<char>& buffer, size_t& buffer_size, const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  vformatToBuffer(buffer, buffer_size, fmt, args);

  va_end(args);
}

std::string formatToString(const char* fmt, ...)
{
  boost::shared_array<char> buffer;
  size_t size = 0;

  va_list args;
  va_start(args, fmt);

  vformatToBuffer(buffer, size, fmt, args);

  va_end(args);

  return std::string(buffer.get(), size);
}

#define INITIAL_BUFFER_SIZE 4096
static boost::mutex g_print_mutex;
static boost::shared_array<char> g_print_buffer(new char[INITIAL_BUFFER_SIZE]);
static size_t g_print_buffer_size = INITIAL_BUFFER_SIZE;
static boost::thread::id g_printing_thread_id;
void print(FilterBase* filter, void* logger_handle, Level level, 
	   const char* file, int line, const char* function, const char* fmt, ...)
{
  if (g_shutting_down)
    return;

  if (g_printing_thread_id == boost::this_thread::get_id())
  {
    fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
    return;
  }

  boost::mutex::scoped_lock lock(g_print_mutex);

  g_printing_thread_id = boost::this_thread::get_id();

  va_list args;
  va_start(args, fmt);

  vformatToBuffer(g_print_buffer, g_print_buffer_size, fmt, args);

  va_end(args);

  bool enabled = true;

  if (filter)
  {
    FilterParams params;
    params.file = file;
    params.function = function;
    params.line = line;
    params.level = level;
    params.logger = logger_handle;
    params.message = g_print_buffer.get();
    enabled = filter->isEnabled(params);
    level = params.level;

    if (!params.out_message.empty())
    {
      size_t msg_size = params.out_message.size();
      if (g_print_buffer_size <= msg_size)
      {
        g_print_buffer_size = msg_size + 1;
        g_print_buffer.reset(new char[g_print_buffer_size]);
      }

      memcpy(g_print_buffer.get(), params.out_message.c_str(), msg_size + 1);
    }
  }

  if (enabled)
  {
    if (level == levels::Error)
    {
      g_last_error_message = g_print_buffer.get();
    }
    try
    {
      ::ros::console::impl::print(logger_handle, level, g_print_buffer.get(), file, function, line);
    }
    catch (std::exception& e)
    {
      fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
    }
  }

  g_printing_thread_id = boost::thread::id();
}

void print(FilterBase* filter, void* logger_handle, Level level, 
	   const std::stringstream& ss, const char* file, int line, const char* function)
{
  if (g_shutting_down)
    return;

  if (g_printing_thread_id == boost::this_thread::get_id())
  {
    fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
    return;
  }

  boost::mutex::scoped_lock lock(g_print_mutex);

  g_printing_thread_id = boost::this_thread::get_id();

  bool enabled = true;
  std::string str = ss.str();

  if (filter)
  {
    FilterParams params;
    params.file = file;
    params.function = function;
    params.line = line;
    params.level = level;
    params.logger = logger_handle;
    params.message = g_print_buffer.get();
    enabled = filter->isEnabled(params);
    level = params.level;

    if (!params.out_message.empty())
    {
      str = params.out_message;
    }
  }

  if (enabled)
  {
    if (level == levels::Error)
    {
      g_last_error_message = str;
    }
    try
    {
      ::ros::console::impl::print(logger_handle, level, str.c_str(), file, function, line);
    }
    catch (std::exception& e)
    {
      fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
    }
  }

  g_printing_thread_id = boost::thread::id();
}

typedef std::vector<LogLocation*> V_LogLocation;
V_LogLocation g_log_locations;
boost::mutex g_locations_mutex;
void registerLogLocation(LogLocation* loc)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  g_log_locations.push_back(loc);
}

void checkLogLocationEnabledNoLock(LogLocation* loc)
{
  loc->logger_enabled_ = ::ros::console::impl::isEnabledFor(loc->logger_, loc->level_);
}

void initializeLogLocation(LogLocation* loc, const std::string& name, Level level)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  if (loc->initialized_)
  {
    return;
  }

  loc->logger_ = ::ros::console::impl::getHandle(name);
  loc->level_ = level;

  g_log_locations.push_back(loc);

  checkLogLocationEnabledNoLock(loc);

  loc->initialized_ = true;
}

void setLogLocationLevel(LogLocation* loc, Level level)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);
  loc->level_ = level;
}

void checkLogLocationEnabled(LogLocation* loc)
{
  boost::mutex::scoped_lock lock(g_locations_mutex);
  checkLogLocationEnabledNoLock(loc);
}

void notifyLoggerLevelsChanged()
{
  boost::mutex::scoped_lock lock(g_locations_mutex);

  V_LogLocation::iterator it = g_log_locations.begin();
  V_LogLocation::iterator end = g_log_locations.end();
  for ( ; it != end; ++it )
  {
    LogLocation* loc = *it;
    checkLogLocationEnabledNoLock(loc);
  }
}

class StaticInit
{
public:
  StaticInit()
  {
    ROSCONSOLE_AUTOINIT;
  }
};
StaticInit g_static_init;


void register_appender(LogAppender* appender)
{
  ros::console::impl::register_appender(appender);
}

void shutdown() 
{
  g_shutting_down = true;
  ros::console::impl::shutdown();
}

bool get_loggers(std::map<std::string, levels::Level>& loggers)
{
  return ros::console::impl::get_loggers(loggers);
}

bool set_logger_level(const std::string& name, levels::Level level)
{
  return ros::console::impl::set_logger_level(name, level);
}

} // namespace console
} // namespace ros
