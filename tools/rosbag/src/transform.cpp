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
#include <iostream>
#include <iomanip>
#include <ros/names.h>

using namespace std;
using namespace rosbag;
namespace fs = boost::filesystem;

ostream& indent(ostream& out)
{
  return out << "    ";
}

ostream& new_field(ostream& out)
{
  return out << left << setw(24u);
}

ostream& start_field(ostream& out)
{
  return out << indent << new_field;
}

bool startsWith(string const& value, string const& key)
{
  return value.find(key, 0) == 0;
}

void usage()
{
  cout << "Usage: transform [general options] [-i|-o file.bag [file operations]]+\n";
  cout << indent << "Applies input operations to each input file, and passes results to all output files.\n";
  cout << indent << "Each output file applies its operations and saves the results.\n";
  cout << "\nGeneral options:\n";
  cout << start_field << "-h, --help" << new_field << "Show these usage information\n";
  cout << start_field << "-q, --quiet" << new_field << "Suppress non-error messages\n";
  cout << start_field << "-n, --no-clobber" << new_field << "Do not overwrite existing output files\n";
  cout << "\nFile type selection:\n";
  cout << start_field << "-i, --input FILE" << new_field << "Read input from bag file FILE.\n";
  cout << start_field << "-o, --output FILE" << new_field << "Write output to bag file FILE\n";
  cout << "\nFile operations:\n";
  cout << start_field << "-s, --start SEC" << new_field << "Start SEC seconds into the bag file\n";
  cout << start_field << "-u, --duration SEC" << new_field << "Keep only SEC seconds from the bag file\n";
  cout << start_field << "--exclude TOPICS" << new_field << "Exclude all given TOPICS (space separated list)\n";
  cout << start_field << "--include TOPICS" << new_field << "Include only given TOPICS (space separated list)\n";
  cout << "\nOutput-only file operations:\n";
  cout << start_field << "-r, --rename TOPICS" << new_field << "Rename given TOPICS (space separated list of OLD:=NEW "
                                                               "entries)\n";
  cout << start_field << "--lz4" << new_field << "Use lz4 compression (lossless, high speed)\n";
  cout << start_field << "-j, --bz2" << new_field << "Use BZ2 compression (lossless, high compression)\n";
  cout.flush();
}

bool parseBagFile(FileOperations& operations, int argc, char* argv[], int& i)
{
  string const arg = argv[i];
  bool const is_input = arg == "-i" || arg == "--input";
  bool const is_output = arg == "-o" || arg == "--output";
  if (is_input || is_output)
  {
    ++i;
    if (i >= argc)
    {
      return false;
    }
    string const next_arg = argv[i];
    auto const path = fs::path(next_arg);
    if (is_input && !fs::exists(path))
    {
      cerr << "Error: File " << next_arg << " does not exist" << endl;
      return false;
    }
    operations.emplace_back(path, is_input ? FileOperation::Input : FileOperation::Output);
  }
  return true;
}

bool parseCompression(FileOperations& operations, string const& arg)
{
  if (arg == "-j" || arg == "--bz2" || arg == "--lz4")
  {
    if (operations.empty())
    {
      cerr << "Error: Please set compression options after output bag files." << endl;
      return false;
    }
    auto& operation = operations.back();
    if (operation.type() != FileOperation::Output)
    {
      cerr << "Error: Compression options can only be set for output files" << endl;
      return false;
    }
    auto const compression = arg == "--lz4" ? rosbag::compression::LZ4 : rosbag::compression::BZ2;
    operation.setCompression(compression);
  }
  return true;
}

bool parseTopicFilter(FileOperations& operations, string const& arg, bool& read_inclusion, bool& read_exclusion,
                      bool& read_rename)
{
  if (read_inclusion || read_exclusion || read_rename)
  {
    if (startsWith(arg, "-"))
    {
      read_inclusion = false;
      read_exclusion = false;
      read_rename = false;
    }
    else
    {
      if (read_inclusion)
      {
        operations.back().addInclusionTopic(arg);
      }
      else if (read_exclusion)
      {
        operations.back().addExclusionTopic(arg);
      }
      else if (read_rename)
      {
        auto& operation = operations.back();
        if (operation.type() != FileOperation::Output)
        {
          cerr << "Error: Topics can only be renamed in output files" << endl;
          return false;
        }
        auto const index = arg.find(":=", 0);
        if (index == string::npos)
        {
          cerr << "Error: Invalid topic rename entry: '" << arg << "'. Please use 'old_name:=new_name'." << endl;
          return false;
        }
        auto const old_name = arg.substr(0u, index);
        auto const new_name = arg.substr(index + 2);
        string error;
        if (!ros::names::validate(new_name, error))
        {
          cerr << "Error: Invalid topic rename entry: '" << arg << "'. Please use 'old_name:=new_name'.\n";
          cerr << "Error message: " << error << endl;
          return false;
        }
        operation.addRenameTopic(old_name, new_name);
      }
    }
  }
  else if (arg == "--exclude")
  {
    if (operations.empty())
    {
      cerr << "Error: Please set topic filters after bag files." << endl;
      return false;
    }
    read_exclusion = true;
  }
  else if (arg == "--include")
  {
    if (operations.empty())
    {
      cerr << "Error: Please set topic filters after bag files." << endl;
      return false;
    }
    read_inclusion = true;
  }
  else if (arg == "-r" || arg == "--rename")
  {
    if (operations.empty())
    {
      cerr << "Error: Please set topic rename after bag files." << endl;
      return false;
    }
    read_rename = true;
  }

  return true;
}

bool parseTime(FileOperations& operations, int argc, char* argv[], int& i)
{
  string const arg = argv[i];
  bool const is_duration = arg == "-u" || arg == "--duration";
  bool const is_start_offset = arg == "-s" || arg == "--start";
  if (is_duration || is_start_offset)
  {
    string const key = is_duration ? "duration" : "start offset";
    if (operations.empty())
    {
      cerr << "Error: Please set " << key << " after bag files." << endl;
      return false;
    }
    double value{ 0.0 };
    ++i;
    if (i >= argc)
    {
      return false;
    }
    string const next_arg = argv[i];
    try
    {
      value = stod(next_arg);
    }
    catch (invalid_argument const& e)
    {
      cerr << "Error: Cannot parse " << key << " value " << next_arg << ": " << e.what() << endl;
      return false;
    }
    catch (out_of_range const& e)
    {
      cerr << "Error: Cannot parse " << key << " value " << next_arg << ": " << e.what() << endl;
      return false;
    }
    if (is_duration)
    {
      operations.back().setDuration(value);
    }
    else if (is_start_offset)
    {
      operations.back().setStartOffset(value);
    }
  }
  return true;
}

bool writeConflict(fs::path const& a, fs::path const& b)
{
  if (fs::exists(a) && fs::exists(b))
  {
    return fs::equivalent(a, b);
  }

  if (fs::exists(a.parent_path()) && fs::exists(b.parent_path()))
  {
    return fs::equivalent(a.parent_path(), b.parent_path()) && a.filename() == b.filename();
  }

  return false;
}

bool allPathsAreUnique(FileOperations& output)
{
  for (size_t i = 0, n = output.size(); i < n; ++i)
  {
    for (size_t j = i + 1; j < n; ++j)
    {
      if (writeConflict(output[i].path(), output[j].path()))
      {
        cerr << "Cannot write to output file " << output[i].path().string() << " multiple times" << endl;
        return false;
      }
    }
  }
  return true;
}

bool parse(int argc, char* argv[], Options& options, FileOperations& input, FileOperations& output)
{
  FileOperations operations;
  if (argc < 2)
  {
    return false;
  }
  bool read_inclusion{ false };
  bool read_exclusion{ false };
  bool read_rename{ false };
  for (int i = 1; i < argc; ++i)
  {
    string const arg = argv[i];
    if (arg == "-h" || arg == "--help")
    {
      input.clear();
      output.clear();
      return true;
    }
    if (arg == "-q" || arg == "--quiet")
    {
      options.setQuiet(true);
    }
    if (arg == "-n" || arg == "--no-clobber")
    {
      options.setClobber(false);
    }
    if (!parseTopicFilter(operations, arg, read_inclusion, read_exclusion, read_rename))
    {
      return false;
    }
    if (!parseBagFile(operations, argc, argv, i))
    {
      return false;
    }
    if (!parseTime(operations, argc, argv, i))
    {
      return false;
    }
    if (!parseCompression(operations, arg))
    {
      return false;
    }
  }

  for (auto const& operation : operations)
  {
    switch (operation.type())
    {
      case FileOperation::Input:
        input.push_back(operation);
        break;
      case FileOperation::Output:
        output.push_back(operation);
        break;
    }
  }

  return allPathsAreUnique(output);
}

int main(int argc, char* argv[])
{
  Options options;
  FileOperations input, output;
  if (parse(argc, argv, options, input, output))
  {
    if (input.empty() || output.empty())
    {
      usage();
      return 0;
    }

    if (!options.clobber())
    {
      for (auto iter = output.begin(); iter != output.end();)
      {
        if (fs::exists(iter->path()))
        {
          iter = output.erase(iter);
        }
        else
        {
          ++iter;
        }
      }
    }
    if (output.empty())
    {
      return 0;
    }

    return Transformer(input, output, options).process() ? 0 : 2;
  }
  usage();
  return 1;
}
