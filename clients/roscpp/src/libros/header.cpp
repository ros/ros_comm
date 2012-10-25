/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "ros/header.h"
#include "ros/transport/transport.h"
#include <ros/console.h>
#include <ros/assert.h>

#include <sstream>

#include <cerrno>

#define SROS_SERIALIZE_PRIMITIVE(ptr, data) { memcpy(ptr, &data, sizeof(data)); ptr += sizeof(data); }
#define SROS_SERIALIZE_BUFFER(ptr, data, data_size) { if (data_size > 0) { memcpy(ptr, data, data_size); ptr += data_size; } }
#define SROS_DESERIALIZE_PRIMITIVE(ptr, data) { memcpy(&data, ptr, sizeof(data)); ptr += sizeof(data); }
#define SROS_DESERIALIZE_BUFFER(ptr, data, data_size) { if (data_size > 0) { memcpy(data, ptr, data_size); ptr += data_size; } }


using namespace std;

namespace ros
{

Header::Header()
: read_map_(new M_string())
{

}

Header::~Header()
{

}

bool Header::parse(const boost::shared_array<uint8_t>& buffer, uint32_t size, std::string& error_msg)
{
  return parse(buffer.get(), size, error_msg);
}

bool Header::parse(uint8_t* buffer, uint32_t size, std::string& error_msg)
{
  uint8_t* buf = buffer;
  while (buf < buffer + size)
  {
    uint32_t len;
    SROS_DESERIALIZE_PRIMITIVE(buf, len);

    if (len > 1000000)
    {
      error_msg = "Received an invalid TCPROS header.  Each element must be prepended by a 4-byte length.";
      ROS_ERROR("%s", error_msg.c_str());

      return false;
    }

    std::string line((char*)buf, len);

    buf += len;

    //printf(":%s:\n", line.c_str());
    size_t eqpos = line.find_first_of("=", 0);
    if (eqpos == string::npos)
    {
      error_msg = "Received an invalid TCPROS header.  Each line must have an equals sign.";
      ROS_ERROR("%s", error_msg.c_str());

      return false;
    }
    string key = line.substr(0, eqpos);
    string value = line.substr(eqpos+1);

    (*read_map_)[key] = value;
  }

  return true;
}

bool Header::getValue(const std::string& key, std::string& value) const
{
  M_string::const_iterator it = read_map_->find(key);
  if (it == read_map_->end())
  {
    return false;
  }

  value = it->second;

  return true;
}

void Header::write(const M_string& key_vals, boost::shared_array<uint8_t>& buffer, uint32_t& size)
{
  // Calculate the necessary size
  size = 0;
  {
    M_string::const_iterator it = key_vals.begin();
    M_string::const_iterator end = key_vals.end();
    for (; it != end; ++it)
    {
      const std::string& key = it->first;
      const std::string& value = it->second;

      size += key.length();
      size += value.length();
      size += 1; // = sign
      size += 4; // 4-byte length
    }
  }

  if (size == 0)
  {
    return;
  }

  buffer.reset(new uint8_t[size]);
  char* ptr = (char*)buffer.get();

  // Write the data
  {
    M_string::const_iterator it = key_vals.begin();
    M_string::const_iterator end = key_vals.end();
    for (; it != end; ++it)
    {
      const std::string& key = it->first;
      const std::string& value = it->second;

      uint32_t len = key.length() + value.length() + 1;
      SROS_SERIALIZE_PRIMITIVE(ptr, len);
      SROS_SERIALIZE_BUFFER(ptr, key.data(), key.length());
      static const char equals = '=';
      SROS_SERIALIZE_PRIMITIVE(ptr, equals);
      SROS_SERIALIZE_BUFFER(ptr, value.data(), value.length());
    }
  }

  ROS_ASSERT(ptr == (char*)buffer.get() + size);
}

}
