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

#ifndef ROS_UDS_EXT_DISABLE

#include "ros/io.h"
#include "ros/transport/transport_uds_stream.h"
#include "ros/poll_set.h"
#include "ros/header.h"
#include "ros/file_log.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <ros/assert.h>
#include <sstream>
#include <boost/atomic.hpp>
#include <boost/bind.hpp>
#include <fcntl.h>
#include <errno.h>
namespace ros
{

bool TransportUDSStream::s_use_keepalive_ = true;

TransportUDSStream::TransportUDSStream(PollSet* poll_set, int flags)
: sock_(ROS_INVALID_SOCKET)
, closed_(false)
, expecting_read_(false)
, expecting_write_(false)
, is_server_(false)
, poll_set_(poll_set)
, flags_(flags)
{
  server_type_ = "/ros-uds-stream-";
}

TransportUDSStream::~TransportUDSStream()
{
  if (!ROS_UDS_EXT_IS_ENABLE(ROS_UDS_EXT_ABSTRACT_SOCK_NAME)) {
    if (!server_uds_path_.empty())
    {
      int err = unlink(server_uds_path_.c_str());
      if(err != 0)
      {
        ROS_ERROR("Unable to remove %s: %s", server_uds_path_.c_str(), strerror(errno));
      }
    }
  }

  ROS_ASSERT_MSG(sock_ == -1, "TransportUDSStream socket [%d] was never closed", sock_);
}

bool TransportUDSStream::setSocket(int sock)
{
  sock_ = sock;
  return initializeSocket();
}

bool TransportUDSStream::setNonBlocking()
{
  if (!(flags_ & SYNCHRONOUS))
  {
	  int result = set_non_blocking(sock_);
	  if ( result != 0 ) {
	      ROS_ERROR("setting socket [%d] as non_blocking failed with error [%d]", sock_, result);
      close();
      return false;
    }
  }

  return true;
}

bool TransportUDSStream::initializeSocket()
{
  ROS_ASSERT(sock_ != ROS_INVALID_SOCKET);

  if (!setNonBlocking())
  {
    return false;
  }

  setKeepAlive(s_use_keepalive_);

  // connect() will set cached_remote_host_ because it already has the UDS path available
  if (cached_remote_host_.empty())
  {
    if (is_server_)
    {
      cached_remote_host_ = "TCPServer Socket";
    }
    else
    {
      std::stringstream ss;
      ss << getClientURI() << " on socket " << sock_;
      cached_remote_host_ = ss.str();
    }
  }

  ROS_ASSERT(poll_set_ || (flags_ & SYNCHRONOUS));
  if (poll_set_)
  {
    ROS_DEBUG("Adding tcp socket [%d] to pollset", sock_);
    poll_set_->addSocket(sock_, boost::bind(&TransportUDSStream::socketUpdate, this, _1), shared_from_this());
  }

  if (!(flags_ & SYNCHRONOUS))
  {
    //enableRead();
  }

  return true;
}

void TransportUDSStream::setKeepAlive(bool use)
{
  if (use)
  {
    int val = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val)) != 0)
    {
      ROS_DEBUG("setsockopt failed to set SO_KEEPALIVE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }
  }
  else
  {
    int val = 0;
    if (setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val)) != 0)
    {
      ROS_DEBUG("setsockopt failed to set SO_KEEPALIVE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }
  }
}

bool TransportUDSStream::connect(const std::string& uds_path)
{
  sock_ = socket(AF_UNIX, SOCK_STREAM, 0);

  if (sock_ == ROS_INVALID_SOCKET)
  {
    ROS_ERROR("socket() failed with error [%s]",  last_socket_error_string());
    return false;
  }

  setNonBlocking();

  struct sockaddr_un sock_addr;
  memset(&sock_addr, 0, sizeof (sock_addr));
  // configure the addr
  sock_addr.sun_family = AF_UNIX;

  int sock_length;
  if (ROS_UDS_EXT_IS_ENABLE(ROS_UDS_EXT_ABSTRACT_SOCK_NAME)) {
    strncpy(&sock_addr.sun_path[1], uds_path.c_str(), uds_path.length());
    sock_length = offsetof(struct sockaddr_un, sun_path) + uds_path.length() + 1;
  } else {
    strncpy(sock_addr.sun_path, uds_path.c_str(), uds_path.length());
    sock_length = offsetof(struct sockaddr_un, sun_path) + uds_path.length();
  }

  int ret = ::connect(sock_, (sockaddr*) &sock_addr, sock_length);
  if (ret != 0)
  {
    ROSCPP_CONN_LOG_DEBUG("Connect to tcpros publisher [%s] failed with error [%d, %s]", uds_path.c_str(), ret, last_socket_error_string());
    close();

    return false;
  }

  std::stringstream ss;
  ss << uds_path << " on socket " << sock_;
  cached_remote_host_ = ss.str();

  if (!initializeSocket())
  {
    return false;
  }

  if (flags_ & SYNCHRONOUS)
  {
    ROSCPP_CONN_LOG_DEBUG("connect() succeeded to [%s] on socket [%d]", uds_path.c_str(), sock_);
  }
  else
  {
    ROSCPP_CONN_LOG_DEBUG("Async connect() in progress to [%s] on socket [%d]", uds_path.c_str(), sock_);
  }

  return true;
}

bool TransportUDSStream::listen(int backlog, const AcceptCallback& accept_cb)
{
  is_server_ = true;
  accept_cb_ = accept_cb;

  sock_ = socket(AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un sock_addr;
  memset(&sock_addr, 0, sizeof(sock_addr));
  sock_addr.sun_family = AF_UNIX;

  int sock_length;
  if (ROS_UDS_EXT_IS_ENABLE(ROS_UDS_EXT_ABSTRACT_SOCK_NAME)) {
    server_uds_path_ = generateServerUDSPath();
    strncpy(&sock_addr.sun_path[1], server_uds_path_.c_str(), server_uds_path_.length());
    sock_length = offsetof(struct sockaddr_un, sun_path) + server_uds_path_.length() + 1;
  } else {
    static boost::atomic_uint32_t uds_counter(0);
    server_uds_path_ = generateServerUDSPath(uds_counter++);
    strncpy(sock_addr.sun_path, server_uds_path_.c_str(), server_uds_path_.length());
    sock_length = offsetof(struct sockaddr_un, sun_path) + server_uds_path_.length();
  }

  if (sock_ <= 0)
  {
    ROS_ERROR("socket() failed with error [%s]", last_socket_error_string());
    return false;
  }

  if (bind(sock_, (struct sockaddr *) &sock_addr, sock_length) < 0)
  {
    ROS_ERROR("bind() failed with error [%s]", last_socket_error_string());
    return false;
  }

  ::listen(sock_, backlog);

  if (!initializeSocket())
  {
    return false;
  }

  if (!(flags_ & SYNCHRONOUS))
  {
    enableRead();
  }

  return true;
}

void TransportUDSStream::close()
{
  Callback disconnect_cb;

  if (!closed_)
  {
    {
      boost::recursive_mutex::scoped_lock lock(close_mutex_);

      if (!closed_)
      {
        closed_ = true;

        ROS_ASSERT(sock_ != ROS_INVALID_SOCKET);

        if (poll_set_)
        {
          poll_set_->delSocket(sock_);
        }

        ::shutdown(sock_, ROS_SOCKETS_SHUT_RDWR);
        if ( close_socket(sock_) != 0 )
        {
          ROS_ERROR("Error closing socket [%d]: [%s]", sock_, last_socket_error_string());
        } else
        {
          ROSCPP_LOG_DEBUG("TCP socket [%d] closed", sock_);
        }
        sock_ = ROS_INVALID_SOCKET;

        disconnect_cb = disconnect_cb_;

        disconnect_cb_ = Callback();
        read_cb_ = Callback();
        write_cb_ = Callback();
        accept_cb_ = AcceptCallback();
      }
    }
  }

  if (disconnect_cb)
  {
    disconnect_cb(shared_from_this());
  }
}

int32_t TransportUDSStream::read(uint8_t* buffer, uint32_t size)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to read on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT(size > 0);

  // never read more than INT_MAX since this is the maximum we can report back with the current return type
  uint32_t read_size = std::min(size, static_cast<uint32_t>(INT_MAX));
  int num_bytes = ::recv(sock_, reinterpret_cast<char*>(buffer), read_size, 0);
  if (num_bytes < 0)
  {
	if ( !last_socket_error_is_would_block() ) // !WSAWOULDBLOCK / !EAGAIN && !EWOULDBLOCK
    {
      ROSCPP_LOG_DEBUG("recv() on socket [%d] failed with error [%s]", sock_, last_socket_error_string());
      close();
    }
    else
    {
      num_bytes = 0;
    }
  }
  else if (num_bytes == 0)
  {
    ROSCPP_LOG_DEBUG("Socket [%d] received 0/%u bytes, closing", sock_, size);
    close();
    return -1;
  }

  return num_bytes;
}

int32_t TransportUDSStream::write(uint8_t* buffer, uint32_t size)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to write on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT(size > 0);

  // never write more than INT_MAX since this is the maximum we can report back with the current return type
  uint32_t writesize = std::min(size, static_cast<uint32_t>(INT_MAX));
  int num_bytes = ::send(sock_, reinterpret_cast<const char*>(buffer), writesize, 0);
  if (num_bytes < 0)
  {
    if ( !last_socket_error_is_would_block() )
    {
      ROSCPP_LOG_DEBUG("send() on socket [%d] failed with error [%s]", sock_, last_socket_error_string());
      close();
    }
    else
    {
      num_bytes = 0;
    }
  }

  return num_bytes;
}

void TransportUDSStream::enableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_read_)
  {
    poll_set_->addEvents(sock_, POLLIN);
    expecting_read_ = true;
  }
}

void TransportUDSStream::disableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_read_)
  {
    poll_set_->delEvents(sock_, POLLIN);
    expecting_read_ = false;
  }
}

void TransportUDSStream::enableWrite()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_write_)
  {
    poll_set_->addEvents(sock_, POLLOUT);
    expecting_write_ = true;
  }
}

void TransportUDSStream::disableWrite()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_write_)
  {
    poll_set_->delEvents(sock_, POLLOUT);
    expecting_write_ = false;
  }
}

TransportUDSStreamPtr TransportUDSStream::accept()
{
  ROS_ASSERT(is_server_);

  int new_sock = ::accept(sock_, NULL, NULL);
  if (new_sock >= 0)
  {
    ROSCPP_LOG_DEBUG("Accepted connection on socket [%d], new socket [%d]", sock_, new_sock);

    TransportUDSStreamPtr transport(boost::make_shared<TransportUDSStream>(poll_set_, flags_));
    if (!transport->setSocket(new_sock))
    {
      ROS_ERROR("Failed to set socket on transport for socket %d", new_sock);
    }

    return transport;
  }
  else
  {
    ROS_ERROR("accept() on socket [%d] failed with error [%s]", sock_,  last_socket_error_string());
  }

  return TransportUDSStreamPtr();
}

void TransportUDSStream::socketUpdate(int events)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);
    if (closed_)
    {
      return;
    }

    // Handle read events before err/hup/nval, since there may be data left on the wire
    if ((events & POLLIN) && expecting_read_)
    {
      if (is_server_)
      {
        // Should not block here, because poll() said that it's ready
        // for reading
        TransportUDSStreamPtr transport = accept();
        if (transport)
        {
          ROS_ASSERT(accept_cb_);
          accept_cb_(transport);
        }
      }
      else
      {
        if (read_cb_)
        {
          read_cb_(shared_from_this());
        }
      }
    }

    if ((events & POLLOUT) && expecting_write_)
    {
      if (write_cb_)
      {
        write_cb_(shared_from_this());
      }
    }
  }

  if((events & POLLERR) ||
     (events & POLLHUP) ||
     (events & POLLNVAL))
  {
    uint32_t error = -1;
    socklen_t len = sizeof(error);
    if (getsockopt(sock_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &len) < 0)
    {
      ROSCPP_LOG_DEBUG("getsockopt failed on socket [%d]", sock_);
    }
    ROSCPP_LOG_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d: %s", sock_, events, strerror(error));

    close();
  }
}

std::string TransportUDSStream::getTransportInfo()
{
  std::stringstream str;
  str << "TCPROS connection to [" << cached_remote_host_ << "]";
  return str.str();
}

std::string TransportUDSStream::getClientURI()
{
  ROS_ASSERT(!is_server_);
  std::stringstream uri;
  uri << "localhost(Unix Domain Socket)";
  return uri.str();
}

} // namespace ros

#endif // ROS_UDS_EXT_DISABLE
