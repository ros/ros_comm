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

#ifndef ROSCPP_HEADER_H
#define ROSCPP_HEADER_H

#include <stdint.h>

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include "common.h"

#include <map>

namespace ros
{

typedef std::map<std::string, std::string> M_string;
typedef boost::shared_ptr<M_string> M_stringPtr;

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;

/**
 * \brief Provides functionality to parse a connection header and retrieve values from it
 */
class ROSCPP_DECL Header
{
public:
  Header();
  ~Header();

  /**
   * \brief Get a value from a parsed header
   * \param key Key value
   * \param value OUT -- value corresponding to the key if there is one
   * \return Returns true if the header had the specified key in it
   */
  bool getValue(const std::string& key, std::string& value) const;
  /**
   * \brief Returns a shared pointer to the internal map used
   */
  M_stringPtr getValues() { return read_map_; }

  /**
   * \brief Parse a header out of a buffer of data
   */
  bool parse(const boost::shared_array<uint8_t>& buffer, uint32_t size, std::string& error_msg);

  /**
   * \brief Parse a header out of a buffer of data
   */
  bool parse(uint8_t* buffer, uint32_t size, std::string& error_msg);

  static void write(const M_string& key_vals, boost::shared_array<uint8_t>& buffer, uint32_t& size);

private:

  M_stringPtr read_map_;
};

}

#endif // ROSCPP_HEADER_H
