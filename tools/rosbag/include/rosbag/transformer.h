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

#ifndef ROSBAG_TRANSFORMER_H
#define ROSBAG_TRANSFORMER_H

#include <fstream>
#include <boost/filesystem.hpp>
#include <vector>
#include <map>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace rosbag
{
class Options
{
public:
  void setQuiet(bool quiet);
  bool quiet() const;
  void setClobber(bool clobber);
  bool clobber() const;

private:
  bool quiet_{ false };
  bool clobber_{ true };
};

class FileOperation
{
public:
  using BagPtr = std::shared_ptr<rosbag::Bag>;
  using Path = boost::filesystem::path;

  enum Type
  {
    Input,
    Output
  };

  FileOperation(Path const& bag_file, Type type);
  Path const& path() const;
  Type type() const;
  void setCompression(rosbag::compression::CompressionType compression);
  rosbag::compression::CompressionType compression() const;
  void addInclusionTopic(std::string const& topic);
  void addExclusionTopic(std::string const& topic);
  void addRenameTopic(std::string const& old_name, std::string const& new_name);
  void setDuration(double duration_secs);
  double duration() const;
  void setStartOffset(double start_offset_secs);
  double startOffset() const;
  void setBag(BagPtr const& bag);
  BagPtr const& bag() const;
  BagPtr& bag();
  bool read(rosbag::ConnectionInfo const* connection_info);
  void write(rosbag::MessageInstance const& message);

private:
  bool keepTopic(std::string const& topic) const;
  bool keepTime(ros::Time const& time);

  Path path_;
  Type type_;
  BagPtr bag_;
  rosbag::CompressionType compression_{ rosbag::compression::Uncompressed };
  std::set<std::string> exclusion_topics_;
  std::set<std::string> inclusion_topics_;
  std::map<std::string, std::string> rename_topics_;
  double duration_{ 0.0 };
  double start_offset_{ 0.0 };
  ros::Time start_time_{ ros::TIME_MAX };
};

using FileOperations = std::vector<FileOperation>;

class Transformer
{
public:
  Transformer(FileOperations const& input, FileOperations const& output, Options const& options);
  bool process();

private:
  bool setupView(rosbag::View& view);
  bool setupOutput();
  bool write(rosbag::View& view);
  bool rename();
  std::ostream& logStream();

  FileOperations input_;
  FileOperations output_;
  std::map<FileOperation::Path, FileOperation::Path> in_place_files_;
  Options options_;
  std::ofstream null_;
  double progress_percent_{ 0.0 };
};

}  // namespace rosbag

#endif
