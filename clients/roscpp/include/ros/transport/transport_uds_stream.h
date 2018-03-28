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

#ifndef ROSCPP_TRANSPORT_UDS_STREAM_H
#define ROSCPP_TRANSPORT_UDS_STREAM_H

#include <ros/types.h>
#include <ros/transport/transport_uds.h>

#include <boost/thread/recursive_mutex.hpp>
#include "ros/io.h"
#include <ros/common.h>

namespace ros
{

class TransportUDSStream;
typedef boost::shared_ptr<TransportUDSStream> TransportUDSStreamPtr;

class PollSet;

/**
 * \brief TCPROS transport with Unix Domain Socket(Stream)
 */
class ROSCPP_DECL TransportUDSStream : public TransportUDS
{
public:
  static bool s_use_keepalive_;

public:
  enum Flags
  {
    SYNCHRONOUS = 1<<0,
  };

  TransportUDSStream(PollSet* poll_set, int flags = 0);
  virtual ~TransportUDSStream();

  /**
   * \brief Connect to a UDS server.
   * \param uds_path The path of Unix Domain Socket to connect to
   * \return Whether or not the connection was successful
   */
  bool connect(const std::string& uds_path);

  /**
   * \brief Returns the URI of the client
   */
  std::string getClientURI();

  typedef boost::function<void(const TransportUDSStreamPtr&)> AcceptCallback;
  /**
   * \brief Start a server socket and listen
   * \param backlog defines the maximum length for the queue of pending connections.  Identical to the backlog parameter to the ::listen function
   * \param accept_cb The function to call when a client socket has connected
   */
  bool listen(int backlog, const AcceptCallback& accept_cb);
  /**
   * \brief Accept a connection on a server socket.  Blocks until a connection is available
   */
  TransportUDSStreamPtr accept();

  void setKeepAlive(bool use);

  // overrides from Transport
  virtual int32_t read(uint8_t* buffer, uint32_t size);
  virtual int32_t write(uint8_t* buffer, uint32_t size);

  virtual void enableWrite();
  virtual void disableWrite();
  virtual void enableRead();
  virtual void disableRead();

  virtual void close();

  virtual std::string getTransportInfo();

  virtual const char* getType() { return "TCPROS"; }

private:
  /**
   * \brief Initializes the assigned socket -- sets it to non-blocking and enables reading
   */
  bool initializeSocket();

  bool setNonBlocking();

  /**
   * \brief Set the socket to be used by this transport
   * \param sock A valid TCP socket
   * \return Whether setting the socket was successful
   */
  bool setSocket(int sock);

  void socketUpdate(int events);

  socket_fd_t sock_;
  bool closed_;
  boost::recursive_mutex close_mutex_;

  bool expecting_read_;
  bool expecting_write_;

  bool is_server_;

  AcceptCallback accept_cb_;

  std::string cached_remote_host_;

  PollSet* poll_set_;
  int flags_;
};

}

#endif // ROSCPP_TRANSPORT_UDS_STREAM_H
