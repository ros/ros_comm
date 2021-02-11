/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Robert Bosch GmbH
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
*********************************************************************/

#include <rosbag/transformer.h>
#include <memory>
#include <boost/bind.hpp>
#include <algorithm>

using namespace std;
namespace fs = boost::filesystem;

namespace rosbag
{
string humanReadableSize(FileOperation::Path const& path)
{
  array<string, 8u> const units{ "B", "K", "M", "G", "T", "P", "E", "Z" };
  double value = fs::file_size(path);
  size_t i = 0;
  size_t const base = 1024;
  for (auto const n = units.size() - 1; value >= base && i < n; ++i)
  {
    value /= base;
  }
  stringstream stream;
  stream << fixed << setprecision(1) << value << units[i];
  return stream.str();
}

void Options::setQuiet(bool quiet)
{
  quiet_ = quiet;
}

bool Options::quiet() const
{
  return quiet_;
}

void Options::setClobber(bool clobber)
{
  clobber_ = clobber;
}

bool Options::clobber() const
{
  return clobber_;
}

FileOperation::FileOperation(Path const& path, Type type) : path_(path), type_(type)
{
  // does nothing
}

FileOperation::Path const& FileOperation::path() const
{
  return path_;
}

FileOperation::Type FileOperation::type() const
{
  return type_;
}

void FileOperation::setCompression(rosbag::compression::CompressionType compression)
{
  compression_ = compression;
}

rosbag::compression::CompressionType FileOperation::compression() const
{
  return compression_;
}

void FileOperation::addInclusionTopic(std::string const& topic)
{
  inclusion_topics_.insert(topic);
}

void FileOperation::addExclusionTopic(std::string const& topic)
{
  exclusion_topics_.insert(topic);
}

void FileOperation::addRenameTopic(std::string const& old_name, std::string const& new_name)
{
  rename_topics_[old_name] = new_name;
}

void FileOperation::setDuration(double duration)
{
  duration_ = duration;
}

double FileOperation::duration() const
{
  return duration_;
}

void FileOperation::setStartOffset(double start_offset)
{
  start_offset_ = start_offset;
}

double FileOperation::startOffset() const
{
  return start_offset_;
}

void FileOperation::setBag(BagPtr const& bag)
{
  bag_ = bag;
}

FileOperation::BagPtr const& FileOperation::bag() const
{
  return bag_;
}

FileOperation::BagPtr& FileOperation::bag()
{
  return bag_;
}

bool FileOperation::keepTopic(std::string const& topic) const
{
  if (!inclusion_topics_.empty())
  {
    return inclusion_topics_.count(topic) == 1;
  }
  if (!exclusion_topics_.empty())
  {
    return exclusion_topics_.count(topic) == 0;
  }
  return true;
}

bool FileOperation::keepTime(ros::Time const& time)
{
  start_time_ = std::min(start_time_, time);
  if (duration_ > 0.0 || start_offset_ > 0.0)
  {
    auto const begin = start_time_ + ros::Duration(start_offset_);
    auto const end = begin + ros::Duration(duration_);
    if (time < begin || time > end)
    {
      return false;
    }
  }
  return true;
}

bool FileOperation::read(rosbag::ConnectionInfo const* connection_info)
{
  if (type_ == Input && bag_ && bag_->getMode() == rosbag::BagMode::Read)
  {
    if (keepTopic(connection_info->topic))
    {
      return true;
    }
  }

  return false;
}

void FileOperation::write(rosbag::MessageInstance const& message)
{
  if (type_ == Output && bag_ && bag_->getMode() != rosbag::BagMode::Read)
  {
    auto iter = rename_topics_.find(message.getTopic());
    auto const topic = iter == rename_topics_.end() ? message.getTopic() : iter->second;
    if (keepTopic(topic) && keepTime(message.getTime()))
    {
      bag()->write(topic, message.getTime(), message, message.getConnectionHeader());
    }
  }
}

Transformer::Transformer(FileOperations const& input, FileOperations const& output, Options const& options)
  : input_(input), output_(output), options_(options)
{
  // does nothing
}

bool Transformer::setupView(rosbag::View& view)
{
  vector<pair<FileOperation::Path, FileOperation::BagPtr>> bags;
  auto& stream = logStream();
  for (auto& file : input_)
  {
    FileOperation::BagPtr bag_ptr;
    for (auto const& bag : bags)
    {
      // Re-use already opened bag file if the same input file
      // is used several times
      if (fs::equivalent(bag.first, file.path()))
      {
        bag_ptr = bag.second;
        break;
      }
    }
    if (!bag_ptr)
    {
      bag_ptr = make_shared<rosbag::Bag>();
      stream << "Opening " << file.path().filename().string();
      stream << " (" << humanReadableSize(file.path()) << ")";
      stream << ", this may take some time." << endl;
      bags.emplace_back(file.path(), bag_ptr);
      try
      {
        bag_ptr->open(file.path().string(), rosbag::bagmode::Read);
      }
      catch (rosbag::BagException const& e)
      {
        cerr << "Cannot open " << file.path().string() << ": " << e.what() << endl;
        return false;
      }
    }
    file.setBag(bag_ptr);

    if (file.duration() > 0.0 || file.startOffset() > 0.0)
    {
      rosbag::View time_range_view(*bag_ptr);
      auto const begin = time_range_view.getBeginTime() + ros::Duration(file.startOffset());
      auto const end = begin + ros::Duration(file.duration());
      view.addQuery(*bag_ptr, bind(&FileOperation::read, file, _1), begin, end);
    }
    else
    {
      view.addQuery(*bag_ptr, bind(&FileOperation::read, file, _1));
    }
  }
  return true;
}

std::ostream& Transformer::logStream()
{
  return options_.quiet() ? null_ : cout;
}

bool Transformer::setupOutput()
{
  auto& stream = logStream();
  for (auto& file : output_)
  {
    FileOperation::Path path = file.path();
    string action = "Creating";
    for (auto const& input : input_)
    {
      if (fs::exists(file.path()) && fs::equivalent(file.path(), input.path()))
      {
        for (int i = 0; i < 100; ++i)
        {
          path = file.path().parent_path() / (file.path().stem().string() + fs::unique_path("_%%%%%%%%.bag").string());
          if (!fs::exists(path))
          {
            in_place_files_[file.path()] = path;
            action = "Updating";
            break;
          }
        }
        if (fs::exists(path))
        {
          cerr << "Unable to create a temporary file to prevent overwriting " << file.path().string() << endl;
          return false;
        }
      }
    }

    stream << action << ' ' << file.path().filename().string() << "." << endl;
    auto bag_ptr = make_shared<rosbag::Bag>();
    bag_ptr->setCompression(file.compression());
    try
    {
      bag_ptr->open(path.string(), rosbag::bagmode::Write);
    }
    catch (rosbag::BagException const& e)
    {
      cerr << "Cannot open " << path.string() << " for writing: " << e.what() << endl;
      return false;
    }
    file.setBag(bag_ptr);
  }
  return true;
}

bool Transformer::write(rosbag::View& view)
{
  auto const size = view.size();
  uint32_t current = 0;
  auto& stream = logStream();
  for (rosbag::MessageInstance const& m : view)
  {
    for (auto& output : output_)
    {
      try
      {
        output.write(m);
      }
      catch (rosbag::BagIOException const& e)
      {
        cerr << "Cannot write to " << output.path().string() << ": " << e.what() << endl;
        return false;
      }
    }

    ++current;
    auto const percent = (100.0 * current) / size;
    if (percent - progress_percent_ > 0.05)
    {
      progress_percent_ = percent;
      string const bags = output_.size() > 1 ? "bags" : "bag";
      stream << "\rWriting " << bags << ", " << fixed << setprecision(1) << progress_percent_ << "%";
      stream.flush();
    }
  }

  for (auto& file : output_)
  {
    stream << "\r" << file.path().filename().string() << " done (" << humanReadableSize(file.path()) << ")" << endl;
  }
  return true;
}

bool Transformer::rename()
{
  for (auto const& target : in_place_files_)
  {
    try
    {
      fs::rename(target.second, target.first);
    }
    catch (fs::filesystem_error const& e)
    {
      cerr << "Unable to rename " << target.second << " to " << target.first << ": " << e.what() << endl;
      return false;
    }
  }
  return true;
}

bool Transformer::process()
{
  bool const reduce_overlap{ true };
  rosbag::View view(reduce_overlap);
  return setupView(view) && setupOutput() && write(view) && rename();
}
}
