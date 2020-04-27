// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Yuki Furuta, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
/*
 * shape_shifter_stamped.h
 * Author: Yuki Furuta <me@furushchev.ru>
 */

#ifndef TOPIC_TOOLS_SHAPE_SHIFTER_STAMPED_H__
#define TOPIC_TOOLS_SHAPE_SHIFTER_STAMPED_H__

#include <std_msgs/Header.h>
#include "topic_tools/shape_shifter.h"

namespace topic_tools
{
class ShapeShifterStamped : public ShapeShifter
{
 public:
  typedef boost::shared_ptr<ShapeShifterStamped> Ptr;
  typedef boost::shared_ptr<ShapeShifterStamped const> ConstPtr;
  void fillHeader() {
    uint8_t buf[size()];
    ros::serialization::OStream stream(buf, size());
    write(stream);
    header.seq = ((uint32_t*)buf)[0];
    header.stamp.sec = ((uint32_t*)buf)[1];
    header.stamp.nsec = ((uint32_t*)buf)[2];
  }

  std_msgs::Header header;
};
}

namespace ros
{
namespace message_traits
{
template<> struct HasHeader<topic_tools::ShapeShifterStamped> : public TrueType {};
template<> struct HasHeader<const topic_tools::ShapeShifterStamped> : public TrueType {};
template<>
struct Header<topic_tools::ShapeShifterStamped,
              typename boost::enable_if<HasHeader<topic_tools::ShapeShifterStamped> >::type>
{
  static std_msgs::Header* pointer(topic_tools::ShapeShifterStamped& m) { return &m.header; }
  static std_msgs::Header const* pointer(const topic_tools::ShapeShifterStamped& m) { return &m.header; }
};
template<>
struct TimeStamp<topic_tools::ShapeShifterStamped,
                 typename boost::enable_if<HasHeader<topic_tools::ShapeShifterStamped> >::type>
{
  static ros::Time* pointer(typename boost::remove_const<topic_tools::ShapeShifterStamped>::type &m) { return &m.header.stamp; }
  static ros::Time const* pointer(const topic_tools::ShapeShifterStamped& m) { return &m.header.stamp; }
  static ros::Time value(const topic_tools::ShapeShifterStamped& m) { return m.header.stamp; }
};
template<>
struct MD5Sum<topic_tools::ShapeShifterStamped>
{
  static const char* value(const topic_tools::ShapeShifterStamped& m) { return m.getMD5Sum().c_str(); }
  static const char* value() { return "*"; }
};
template<>
struct DataType<topic_tools::ShapeShifterStamped>
{
  static const char* value(const topic_tools::ShapeShifterStamped& m) { return m.getDataType().c_str(); }
  static const char* value() { return "*"; }
};

} // message_traits


namespace serialization
{

template<>
struct Serializer<topic_tools::ShapeShifterStamped>
{
  template<typename Stream>
  inline static void write(Stream& stream, const topic_tools::ShapeShifterStamped& m) {
    m.write(stream);
  }

  template<typename Stream>
  inline static void read(Stream& stream, topic_tools::ShapeShifterStamped& m)
  {
    m.read(stream);
    m.fillHeader();
  }

  inline static uint32_t serializedLength(const topic_tools::ShapeShifterStamped& m) {
    return m.size();
  }
};
template<>
struct PreDeserialize<topic_tools::ShapeShifterStamped>
{
  static void notify(const PreDeserializeParams<topic_tools::ShapeShifterStamped>& params)
  {
    std::string md5      = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def  = (*params.connection_header)["message_definition"];
    std::string latching  = (*params.connection_header)["latching"];

    params.message->morph(md5, datatype, msg_def, latching);
  }
};
} // serialization
} // ros

#endif // TOPIC_TOOLS_SHAPE_SHIFTER_STAMPED_H__
