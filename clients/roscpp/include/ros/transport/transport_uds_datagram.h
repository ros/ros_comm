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

#ifndef ROSCPP_TRANSPORT_UDS_DATAGRAM_H
#define ROSCPP_TRANSPORT_UDS_DATAGRAM_H

#include <ros/types.h>
#include <ros/transport/transport_uds.h>

#include <boost/thread/mutex.hpp>
#include "ros/io.h"
#include <ros/common.h>

namespace ros
{

class TransportUDSDatagram;
typedef boost::shared_ptr<TransportUDSDatagram> TransportUDSDatagramPtr;

class PollSet;

#define ROS_UDP_DATA0 0
#define ROS_UDP_DATAN 1
#define ROS_UDP_PING 2
#define ROS_UDP_ERR 3
typedef struct TransportUDSDatagramHeader {
  uint32_t connection_id_;
  uint8_t op_;
  uint8_t message_id_;
  uint16_t block_;
} TransportUDSDatagramHeader;

/**
 * \brief UDPROS transport with Unix Domain Socket(Datagram)
 */
class ROSCPP_DECL TransportUDSDatagram : public TransportUDS
{
public:
  enum Flags
  {
    SYNCHRONOUS = 1<<0,
  };

  TransportUDSDatagram(PollSet* poll_set, int flags = 0, int max_datagram_size = 0);
  virtual ~TransportUDSDatagram();

  /**
   * \brief Connect to a remote host.
   * \param uds_path The UDS path to connect to
   * \return Whether or not the connection was successful
   */
  bool connect(const std::string& uds_path, int conn_id);

  /**
   * \brief Returns the URI of the client
   */
  std::string getClientURI();

  /**
   * \brief Start a server socket and listen
   */
  bool createIncoming(bool is_server);
  /**
   * \brief Create a connection to a server socket.
   */
  TransportUDSDatagramPtr createOutgoing(const std::string& uds_path, int conn_id, int max_datagram_size);

  // overrides from Transport
  virtual int32_t read(uint8_t* buffer, uint32_t size);
  virtual int32_t write(uint8_t* buffer, uint32_t size);

  virtual void enableWrite();
  virtual void disableWrite();
  virtual void enableRead();
  virtual void disableRead();

  virtual void close();

  virtual std::string getTransportInfo();

  virtual bool requiresHeader() {return false;}

  virtual const char* getType() {return "UDPROS";}

  int getMaxDatagramSize() const {return max_datagram_size_;}

private:
  /**
   * \brief Initializes the assigned socket -- sets it to non-blocking and enables reading
   */
  bool initializeSocket();

  /**
   * \brief Set the socket to be used by this transport
   * \param sock A valid UDP socket
   * \return Whether setting the socket was successful
   */
  bool setSocket(int sock);

  void socketUpdate(int events);

  socket_fd_t sock_;
  bool closed_;
  boost::mutex close_mutex_;

  bool expecting_read_;
  bool expecting_write_;

  bool is_server_;

  std::string cached_remote_host_;

  PollSet* poll_set_;
  int flags_;

  uint32_t connection_id_;
  uint8_t current_message_id_;
  uint16_t total_blocks_;
  uint16_t last_block_;

  uint32_t max_datagram_size_;

  uint8_t* data_buffer_;
  uint8_t* data_start_;
  uint32_t data_filled_;

  uint8_t* reorder_buffer_;
  uint8_t* reorder_start_;
  TransportUDSDatagramHeader reorder_header_;
  uint32_t reorder_bytes_;
};

}

#endif // ROSCPP_TRANSPORT_UDS_DATAGRAM_H
