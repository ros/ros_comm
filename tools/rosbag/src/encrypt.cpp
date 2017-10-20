/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Open Source Robotics Foundation
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

#include <iostream>

#include <boost/scoped_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>

#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace po = boost::program_options;

struct EncryptorOptions
{
    EncryptorOptions() : quiet(false), compression(rosbag::compression::Uncompressed) { }

    void buildOutbagName();

    bool quiet;
    std::string plugin;
    std::string param;
    rosbag::CompressionType compression;
    std::string inbag;
    std::string outbag;
};

void EncryptorOptions::buildOutbagName() {
    if (!outbag.empty()) {
        return;
    }
    if (inbag.empty()) {
        throw ros::Exception("Input bag is not specified.");
    }
    std::string::size_type pos = inbag.find_last_of('.');
    if (pos == std::string::npos) {
        throw ros::Exception("Input bag name has no extension.");
    }
    outbag = inbag.substr(0, pos) + std::string(".out") + inbag.substr(pos);
}

//! Parse the command-line arguments for encrypt options
EncryptorOptions parseOptions(int argc, char** argv) {
    EncryptorOptions opts;

    po::options_description desc("Allowed options");

    desc.add_options()
      ("help,h",   "produce help message")
      ("quiet,q",  "suppress console output")
      ("plugin,p", po::value<std::string>()->default_value("rosbag/AesCbcEncryptor"), "encryptor name")
      ("param,r",  po::value<std::string>()->default_value("*"),                      "encryptor parameter")
      ("bz2,j",    "use BZ2 compression")
      ("lz4",      "use lz4 compression")
      ("inbag",    po::value<std::string>(),                                          "bag file to encrypt")
      ("outbag,o", po::value<std::string>(),                                          "bag file encrypted")
      ;

    po::positional_options_description p;
    p.add("inbag", -1);

    po::variables_map vm;

    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    } catch (boost::program_options::invalid_command_line_syntax& e)
    {
      throw ros::Exception(e.what());
    } catch (boost::program_options::unknown_option& e)
    {
      throw ros::Exception(e.what());
    }

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(0);
    }

    if (vm.count("quiet"))
      opts.quiet = true;
    if (vm.count("plugin"))
      opts.plugin = vm["plugin"].as<std::string>();
    if (vm.count("param"))
      opts.param = vm["param"].as<std::string>();
    if (vm.count("bz2"))
      opts.compression = rosbag::compression::BZ2;
    if (vm.count("lz4"))
      opts.compression = rosbag::compression::LZ4;
    if (vm.count("inbag"))
    {
      opts.inbag = vm["inbag"].as<std::string>();
    } else {
      throw ros::Exception("You must specify bag to encrypt.");
    }
    if (vm.count("outbag"))
      opts.outbag = vm["outbag"].as<std::string>();
    opts.buildOutbagName();

    return opts;
}

std::string getStringCompressionType(rosbag::CompressionType compression) {
    switch(compression) {
    case rosbag::compression::Uncompressed: return "none";
    case rosbag::compression::BZ2: return "bz2";
    case rosbag::compression::LZ4: return "lz4";
    default: return "Unknown";
    }
}

int encrypt(EncryptorOptions const& options) {
    if (!options.quiet) {
        std::cout << "Output bag:  " << options.outbag << "\n";
        std::cout << "Encryption:  " << options.plugin << ":" << options.param << "\n";
        std::cout << "Compression: " << getStringCompressionType(options.compression) << "\n";
    }
    rosbag::Bag inbag(options.inbag, rosbag::bagmode::Read);
    rosbag::Bag outbag(options.outbag, rosbag::bagmode::Write);
    // Compression type is per chunk, and cannot be retained.
    // If chunk-by-chunk encryption is implemented, compression type could be honored.
    outbag.setEncryptorPlugin(options.plugin, options.param);
    outbag.setCompression(options.compression);
    rosbag::View view(inbag);
    boost::scoped_ptr<boost::progress_display> progress;
    if (!options.quiet)
        progress.reset(new boost::progress_display(view.size(), std::cout, "Progress:\n  ", "  ", "  "));
    for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
        outbag.write(it->getTopic(), it->getTime(), *it, it->getConnectionHeader());
        if (progress)
            ++(*progress);
    }
    outbag.close();
    inbag.close();
    return 0;
}

int main(int argc, char** argv) {
    // Parse the command-line options
    EncryptorOptions opts;
    try {
        opts = parseOptions(argc, argv);
    }
    catch (ros::Exception const& ex) {
        ROS_ERROR("Error reading options: %s", ex.what());
        return 1;
    }
    catch(boost::regex_error const& ex) {
        ROS_ERROR("Error reading options: %s\n", ex.what());
        return 1;
    }

    return encrypt(opts);
}
