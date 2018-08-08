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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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
#include <cstring>
#include <cstdlib>

#include "ros/ros.h"
#include "ros/file_log.h"

#ifdef WIN32
  #ifdef ERROR
    // ach, windows.h polluting everything again,
        // clashes with autogenerated rosgraph_msgs/Log.h
    #undef ERROR
  #endif
#endif
#include "rosgraph_msgs/Log.h"

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b rosout logs messages sent to the /rosout topic,
 * which is a system-wide string logging mechanism.
 */

/**
 * the rosout node subscribes to /rosout, logs the messages to file, and re-broadcasts the messages to /rosout_agg
 */
class Rosout
{
public:
  std::string log_file_name_;
  FILE* handle_;
  size_t max_file_size_;
  size_t current_file_size_;
  size_t max_backup_index_;
  size_t current_backup_index_;
  ros::NodeHandle node_;
  ros::Subscriber rosout_sub_;
  ros::Publisher agg_pub_;
  bool omit_topics_;

  Rosout() :
    log_file_name_(ros::file_log::getLogDirectory() + "/rosout.log"),
    handle_(NULL),
    max_file_size_(100*1024*1024),
    current_file_size_(0),
    max_backup_index_(10),
    current_backup_index_(0),
    omit_topics_(false)
  {
    init();
  }

  void init()
  {
    const char* disable_file_logging_env = getenv("ROSOUT_DISABLE_FILE_LOGGING");
    std::string disable_file_logging(disable_file_logging_env ? disable_file_logging_env : "");
    std::transform(
      disable_file_logging.begin(),
      disable_file_logging.end(),
      disable_file_logging.begin(),
      [](char c){ return std::tolower(c); });
    if (disable_file_logging.empty()  // Not set or set to empty string.
      || disable_file_logging == "0"
      || disable_file_logging == "false"
      || disable_file_logging == "off"
      || disable_file_logging == "no")
    {
      handle_ = fopen(log_file_name_.c_str(), "w");

      if (handle_ == 0)
      {
        std::cerr << "Error opening rosout log file '" << log_file_name_.c_str() << "': " << strerror(errno);
      }
      else
      {
        std::cout << "logging to " << log_file_name_.c_str() << std::endl;

        std::stringstream ss;
        ss <<  "\n\n" << ros::Time::now() << "  Node Startup\n";
        int written = fprintf(handle_, "%s", ss.str().c_str());
        if (written < 0)
        {
          std::cerr << "Error writting to rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_)) << std::endl;
        }
        else if (written > 0)
        {
          current_file_size_ += written;
          if (fflush(handle_))
          {
            std::cerr << "Error flushing rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_));
          }
        }
      }
    }

    agg_pub_ = node_.advertise<rosgraph_msgs::Log>("/rosout_agg", 0);
    std::cout << "re-publishing aggregated messages to /rosout_agg" << std::endl;

    rosout_sub_ = node_.subscribe("/rosout", 0, &Rosout::rosoutCallback, this);
    std::cout << "subscribed to /rosout" << std::endl;
  }

  void rosoutCallback(const rosgraph_msgs::Log::ConstPtr& msg)
  {
    agg_pub_.publish(msg);

    if (!handle_)
    {
      return;
    }

    std::stringstream ss;
    ss << msg->header.stamp << " ";
    switch (msg->level)
    {
    case rosgraph_msgs::Log::FATAL:
      ss << "FATAL ";
      break;
    case rosgraph_msgs::Log::ERROR:
      ss << "ERROR ";
      break;
    case rosgraph_msgs::Log::WARN:
      ss << "WARN ";
      break;
    case rosgraph_msgs::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case rosgraph_msgs::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << msg->level << " ";
    }

    ss << msg->name << " ";
    ss << "[" << msg->file << ":" << msg->line << "(" << msg->function << ")] ";

    // check parameter server for omit_topics flag and set class member
    node_.getParamCached("/rosout/omit_topics", omit_topics_);

    if (!omit_topics_)
    {
      ss << "[topics: ";
      std::vector<std::string>::const_iterator it = msg->topics.begin();
      std::vector<std::string>::const_iterator end = msg->topics.end();
      for ( ; it != end; ++it )
      {
        const std::string& topic = *it;

        if ( it != msg->topics.begin() )
        {
          ss << ", ";
        }

        ss << topic;
      }
      ss << "] ";
    }

    ss << msg->msg;
    ss << "\n";
    int written = fprintf(handle_, "%s", ss.str().c_str());
    if (written < 0)
    {
      std::cerr << "Error writting to rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_)) << std::endl;
    }
    else if (written > 0)
    {
      current_file_size_ += written;
      if (fflush(handle_))
      {
        std::cerr << "Error flushing rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_));
      }

      // check for rolling
      if (current_file_size_ > max_file_size_)
      {
        std::cout << "rosout log file " << log_file_name_.c_str() << " reached max size, rotating log files" << std::endl;
        if (fclose(handle_))
        {
          std::cerr << "Error closing rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_)) << std::endl;
        }
        if (current_backup_index_ == max_backup_index_)
        {
          std::stringstream backup_file_name;
          backup_file_name << log_file_name_ << "." << max_backup_index_;
          int rc = remove(backup_file_name.str().c_str());
          if (rc != 0)
          {
            std::cerr << "Error deleting oldest rosout log file '" << backup_file_name.str().c_str() << "': " << strerror(errno) << std::endl;
          }
        }
        std::size_t i = std::min(max_backup_index_, current_backup_index_ + 1);
        while (i > 0)
        {
          std::stringstream current_file_name;
          current_file_name << log_file_name_;
          if (i > 1)
          {
            current_file_name << "." << (i - 1);
          }
          std::stringstream rotated_file_name;
          rotated_file_name << log_file_name_ << "." << i;
          int rc = rename(current_file_name.str().c_str(), rotated_file_name.str().c_str());
          if (rc != 0)
          {
            std::cerr << "Error rotating rosout log file '" << current_file_name.str().c_str() << "' to '" << rotated_file_name.str().c_str() << "': " << strerror(errno) << std::endl;
          }
          --i;
        }
        if (current_backup_index_ < max_backup_index_)
        {
          ++current_backup_index_;
        }
        handle_ = fopen(log_file_name_.c_str(), "w");
        if (handle_ == 0)
        {
          std::cerr << "Error opening rosout log file '" << log_file_name_.c_str() << "': " << strerror(errno);
        }
        current_file_size_ = 0;
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosout", ros::init_options::NoRosout);
  ros::NodeHandle n;
  Rosout r;

  ros::spin();

  return 0;
}

