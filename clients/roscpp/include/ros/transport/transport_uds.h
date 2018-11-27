/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Open Source Robotics Foundation, Inc.
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

#ifndef ROSCPP_TRANSPORT_UDS_H
#define ROSCPP_TRANSPORT_UDS_H

#include <ros/transport/transport.h>

/*! @name UDS Feature Bit Field
    Unix Domain Socket Feature Bit Field controlled by environmental value from user.
*/
/* @{ */
#define ROS_UDS_EXT_ABSTRACT_SOCK_NAME   0x00000001    /*!< enable abstract named socket */
/* @} */

#define ROS_UDS_EXT_IS_ENABLE(feature)   (TransportUDS::s_uds_feature_ & feature)

namespace ros
{

/**
 * \brief Abstract base class that allows abstraction of the transport type, eg. Unix domain socket(stream or datagram)
 */
class TransportUDS : public Transport
{
public:
  static uint32_t s_uds_feature_;

  /**
   * \brief Returns the server UDS path this transport is using with
   */
  const std::string getServerUDSPath() { return server_uds_path_; }

protected:
  /**
   * \brief Generate abstrace named socket".
   */
  const std::string generateServerUDSPath();
  /**
   * \brief Generate a string of UDS path like "${TMP}/ros-uds-[stream|datagram]-${PID}-${COUNTER}", the default value of ${TMP} is "/tmp".
   */
  const std::string generateServerUDSPath(uint32_t counter);

  std::string server_type_;
  std::string server_uds_path_;
};

}

#endif // ROSCPP_TRANSPORT_UDS_H
