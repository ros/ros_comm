#include "ros/console.h"

#include <glog/logging.h>

namespace ros
{
namespace console
{
namespace impl
{

std::vector<std::pair<std::string, levels::Level> > rosconsole_glog_log_levels;
LogAppender* rosconsole_glog_appender = 0;

void initialize()
{
  google::InitGoogleLogging("rosconsole");
}

std::string getName(void* handle);

void print(void* handle, ::ros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  // still printing to console
  ::ros::console::backend::print(0, level, str, file, function, line);

  // pass log message to appender
  if(rosconsole_glog_appender)
  {
    rosconsole_glog_appender->log(level, str, file, function, line);
  }

  google::LogSeverity glog_level;
  if(level == ::ros::console::levels::Info)
  {
    glog_level = google::GLOG_INFO;
  }
  else if(level == ::ros::console::levels::Warn)
  {
    glog_level = google::GLOG_WARNING;
  }
  else if(level == ::ros::console::levels::Error)
  {
    glog_level = google::GLOG_ERROR;
  }
  else if(level == ::ros::console::levels::Fatal)
  {
    glog_level = google::GLOG_FATAL;
  }
  else
  {
    // ignore debug
    return;
  }
  std::string name = getName(handle);
  google::LogMessage(file, line, glog_level).stream() << name << ": " << str;
}

bool isEnabledFor(void* handle, ::ros::console::Level level)
{
  size_t index = (size_t)handle;
  if(index < rosconsole_glog_log_levels.size())
  {
    return level >= rosconsole_glog_log_levels[index].second;
  }
  return false;
}

void* getHandle(const std::string& name)
{
  size_t count = rosconsole_glog_log_levels.size();
  for(size_t index = 0; index < count; index++)
  {
    if(name == rosconsole_glog_log_levels[index].first)
    {
      return (void*)index;
    }
    index++;
  }
  // add unknown names on demand with default level
  rosconsole_glog_log_levels.push_back(std::pair<std::string, levels::Level>(name, ::ros::console::levels::Info));
  return (void*)(rosconsole_glog_log_levels.size() - 1);
}

std::string getName(void* handle)
{
  size_t index = (size_t)handle;
  if(index < rosconsole_glog_log_levels.size())
  {
    return rosconsole_glog_log_levels[index].first;
  }
  return "";
}

void register_appender(LogAppender* appender)
{
  rosconsole_glog_appender = appender;
}

void shutdown()
{}

bool get_loggers(std::map<std::string, levels::Level>& loggers)
{
  for(std::vector<std::pair<std::string, levels::Level> >::const_iterator it = rosconsole_glog_log_levels.begin(); it != rosconsole_glog_log_levels.end(); it++)
  {
    loggers[it->first] = it->second;
  }
  return true;
}

bool set_logger_level(const std::string& name, levels::Level level)
{
  for(std::vector<std::pair<std::string, levels::Level> >::iterator it = rosconsole_glog_log_levels.begin(); it != rosconsole_glog_log_levels.end(); it++)
  {
    if(name == it->first)
    {
      it->second = level;
      return true;
    }
  }
  return false;
}

} // namespace impl
} // namespace console
} // namespace ros
