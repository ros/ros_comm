/*
 * Sony CONFIDENTIAL
 *
 * Copyright 2018 Sony Corporation
 *
 * DO NOT COPY AND/OR REDISTRIBUTE WITHOUT PERMISSION.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef ROSCPP_TRANSPORT_UDS_H
#define ROSCPP_TRANSPORT_UDS_H

#ifndef ROS_UDS_EXT_DISABLE

#include <ros/transport/transport.h>

namespace ros
{

/**
 * \brief Abstract base class that allows abstraction of the transport type, eg. Unix domain socket(stream or datagram)
 */
class TransportUDS : public Transport
{
public:
  static bool s_use_uds_;

  /**
   * \brief Returns the server UDS path this transport is using with
   */
  const std::string getServerUDSPath() { return server_uds_path_; }

protected:
  /**
   * \brief Generate a string of UDS path like "${TMP}/ros-uds-[stream|datagram]-${PID}-${COUNTER}", the default value of ${TMP} is "/tmp".
   */
  const std::string generateServerUDSPath(uint32_t counter);

  std::string server_type_;
  std::string server_uds_path_;
};

}

#endif // ROS_UDS_EXT_DISABLE
#endif // ROSCPP_TRANSPORT_UDS_H
