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

#include "ros/transport/transport_uds_datagram.h"
#include "ros/poll_set.h"
#include "ros/file_log.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <ros/assert.h>
#include <boost/atomic.hpp>
#include <boost/bind.hpp>

#include <fcntl.h>
#if defined(__APPLE__)
  // For readv() and writev()
  #include <sys/types.h>
  #include <sys/uio.h>
  #include <unistd.h>
#elif defined(__ANDROID__)
  // For readv() and writev() on ANDROID
  #include <sys/uio.h>
#endif

namespace ros
{

TransportUDSDatagram::TransportUDSDatagram(PollSet* poll_set, int flags, int max_datagram_size)
: sock_(-1)
, closed_(false)
, expecting_read_(false)
, expecting_write_(false)
, is_server_(false)
, poll_set_(poll_set)
, flags_(flags)
, connection_id_(0)
, current_message_id_(0)
, total_blocks_(0)
, last_block_(0)
, max_datagram_size_(max_datagram_size)
, data_filled_(0)
, reorder_buffer_(0)
, reorder_bytes_(0)
{
  // This may eventually be machine dependent
  if (max_datagram_size_ == 0)
    max_datagram_size_ = 1500;
  reorder_buffer_ = new uint8_t[max_datagram_size_];
  reorder_start_ = reorder_buffer_;
  data_buffer_ = new uint8_t[max_datagram_size_];
  data_start_ = data_buffer_;
  server_type_ = "/ros-uds-datagram-";
}

TransportUDSDatagram::~TransportUDSDatagram()
{
  if (!server_uds_path_.empty())
  {
    int err = unlink(server_uds_path_.c_str());
    if(err != 0)
    {
        ROS_ERROR("Unable to remove %s: %s", server_uds_path_.c_str(), strerror(errno));
    }
  }

  ROS_ASSERT_MSG(sock_ == ROS_INVALID_SOCKET, "TransportUDSDatagram socket [%d] was never closed", sock_);
  delete [] reorder_buffer_;
  delete [] data_buffer_;
}

bool TransportUDSDatagram::setSocket(int sock)
{
  sock_ = sock;
  return initializeSocket();
}

void TransportUDSDatagram::socketUpdate(int events)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if((events & POLLERR) ||
     (events & POLLHUP) ||
     (events & POLLNVAL))
  {
    ROSCPP_LOG_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d", sock_, events);
    close();
  }
  else
  {
    if ((events & POLLIN) && expecting_read_)
    {
      if (read_cb_)
      {
        read_cb_(shared_from_this());
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

}

std::string TransportUDSDatagram::getTransportInfo()
{
  std::stringstream str;
  str << "UDPROS connection to [" << cached_remote_host_ << "]";
  return str.str();
}

bool TransportUDSDatagram::connect(const std::string& uds_path, int connection_id)
{
  sock_ = socket(AF_UNIX, SOCK_DGRAM, 0);
  connection_id_ = connection_id;

  if (sock_ == ROS_INVALID_SOCKET)
  {
    ROS_ERROR("socket() failed with error [%s]",  last_socket_error_string());
    return false;
  }

  struct sockaddr_un sock_addr;
  memset(&sock_addr, 0, sizeof(sock_addr));
  sock_addr.sun_family = AF_UNIX;

  strncpy(sock_addr.sun_path, uds_path.c_str(), uds_path.length());
  int sock_length = offsetof(struct sockaddr_un, sun_path) + uds_path.length();

  if (::connect(sock_, (sockaddr *)&sock_addr, sock_length))
  {
    ROSCPP_LOG_DEBUG("Connect to udpros host [%s] failed with error [%s]", uds_path.c_str(), last_socket_error_string());
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

  ROSCPP_LOG_DEBUG("Connect succeeded to [%s] on socket [%d]", uds_path.c_str(), sock_);

  return true;
}

bool TransportUDSDatagram::createIncoming(bool is_server)
{
  is_server_ = is_server;

  sock_ = socket(AF_UNIX, SOCK_DGRAM, 0);

  if (sock_ <= 0)
  {
    ROS_ERROR("socket() failed with error [%s]", last_socket_error_string());
    return false;
  }

  struct sockaddr_un sock_addr;
  memset(&sock_addr, 0, sizeof(sock_addr));
  sock_addr.sun_family = AF_UNIX;

  static boost::atomic_uint32_t uds_counter(0);
  server_uds_path_ = generateServerUDSPath(uds_counter++);
  strncpy(sock_addr.sun_path, server_uds_path_.c_str(), server_uds_path_.length());
  int sock_length = offsetof(struct sockaddr_un, sun_path) + server_uds_path_.length();

  if (bind(sock_, (sockaddr *)&sock_addr, sock_length) < 0)
  {
    ROS_ERROR("bind() failed with error [%s]",  last_socket_error_string());
    return false;
  }

  ROSCPP_LOG_DEBUG("UDPROS server listening on path [%s]", server_uds_path_.c_str());

  if (!initializeSocket())
  {
    return false;
  }

  enableRead();

  return true;
}

bool TransportUDSDatagram::initializeSocket()
{
  ROS_ASSERT(sock_ != ROS_INVALID_SOCKET);

  if (!(flags_ & SYNCHRONOUS))
  {
	  int result = set_non_blocking(sock_);
	  if ( result != 0 ) {
	      ROS_ERROR("setting socket [%d] as non_blocking failed with error [%d]", sock_, result);
      close();
      return false;
    }
  }

  ROS_ASSERT(poll_set_ || (flags_ & SYNCHRONOUS));
  if (poll_set_)
  {
    poll_set_->addSocket(sock_, boost::bind(&TransportUDSDatagram::socketUpdate, this, _1), shared_from_this());
  }

  return true;
}

void TransportUDSDatagram::close()
{
  Callback disconnect_cb;

  if (!closed_)
  {
    {
      boost::mutex::scoped_lock lock(close_mutex_);

      if (!closed_)
      {
        closed_ = true;

        ROSCPP_LOG_DEBUG("UDP socket [%d] closed", sock_);

        ROS_ASSERT(sock_ != ROS_INVALID_SOCKET);

        if (poll_set_)
        {
          poll_set_->delSocket(sock_);
        }

        if ( close_socket(sock_) != 0 )
        {
          ROS_ERROR("Error closing socket [%d]: [%s]", sock_, last_socket_error_string());
        }

        sock_ = ROS_INVALID_SOCKET;

        disconnect_cb = disconnect_cb_;

        disconnect_cb_ = Callback();
        read_cb_ = Callback();
        write_cb_ = Callback();
      }
    }
  }

  if (disconnect_cb)
  {
    disconnect_cb(shared_from_this());
  }
}

int32_t TransportUDSDatagram::read(uint8_t* buffer, uint32_t size)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);
    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to read on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  uint32_t bytes_read = 0;

  while (bytes_read < size)
  {
    TransportUDSDatagramHeader header;

    // Get the data either from the reorder buffer or the socket
    // copy_bytes will contain the read size.
    // from_previous is true if the data belongs to the previous UDP datagram.
    uint32_t copy_bytes = 0;
    bool from_previous = false;
    if (reorder_bytes_)
    {
      if (reorder_start_ != reorder_buffer_)
      {
        from_previous = true;
      }

      copy_bytes = std::min(size - bytes_read, reorder_bytes_);
      header = reorder_header_;
      memcpy(buffer + bytes_read, reorder_start_, copy_bytes);
      reorder_bytes_ -= copy_bytes;
      reorder_start_ += copy_bytes;
    }
    else
    {
      if (data_filled_ == 0)
      {
        ssize_t num_bytes;
        struct iovec iov[2];
        iov[0].iov_base = &header;
        iov[0].iov_len = sizeof(header);
        iov[1].iov_base = data_buffer_;
        iov[1].iov_len = max_datagram_size_ - sizeof(header);
        // Read a datagram with header
        num_bytes = readv(sock_, iov, 2);

        if (num_bytes < 0)
        {
          if ( last_socket_error_is_would_block() )
          {
            num_bytes = 0;
            break;
          }
          else
          {
            ROSCPP_LOG_DEBUG("readv() failed with error [%s]",  last_socket_error_string());
            close();
            break;
          }
        }
        else if (num_bytes == 0)
        {
          ROSCPP_LOG_DEBUG("Socket [%d] received 0/%d bytes, closing", sock_, size);
          close();
          return -1;
        }
        else if (num_bytes < (unsigned) sizeof(header))
        {
          ROS_ERROR("Socket [%d] received short header (%d bytes): %s", sock_, int(num_bytes),  last_socket_error_string());
          close();
          return -1;
        }

		num_bytes -= sizeof(header);
        data_filled_ = num_bytes;
        data_start_ = data_buffer_;
      }
      else
      {
        from_previous = true;
      }

      copy_bytes = std::min(size - bytes_read, data_filled_);
      // Copy from the data buffer, whether it has data left in it from a previous datagram or
      // was just filled by readv()
      memcpy(buffer + bytes_read, data_start_, copy_bytes);
      data_filled_ -= copy_bytes;
      data_start_ += copy_bytes;
    }


    if (from_previous)
    {
      // We are simply reading data from the last UDP datagram, nothing to
      // parse
      bytes_read += copy_bytes;
    }
    else
    {
      // This datagram is new, process header
      switch (header.op_)
      {
        case ROS_UDP_DATA0:
          if (current_message_id_)
          {
            ROS_DEBUG("Received new message [%d:%d], while still working on [%d] (block %d of %d)", header.message_id_, header.block_, current_message_id_, last_block_ + 1, total_blocks_);
            reorder_header_ = header;

            // Copy the entire data buffer to the reorder buffer, as we will
            // need to replay this UDP datagram in the next call.
            reorder_bytes_ = data_filled_ + (data_start_ - data_buffer_);
            memcpy(reorder_buffer_, data_buffer_, reorder_bytes_);
            reorder_start_ = reorder_buffer_;
            current_message_id_ = 0;
            total_blocks_ = 0;
            last_block_ = 0;

            data_filled_ = 0;
            data_start_ = data_buffer_;
            return -1;
          }
          total_blocks_ = header.block_;
          last_block_ = 0;
          current_message_id_ = header.message_id_;
          break;
        case ROS_UDP_DATAN:
          if (header.message_id_ != current_message_id_)
          {
            ROS_DEBUG("Message Id mismatch: %d != %d", header.message_id_, current_message_id_);
            data_filled_ = 0; // discard datagram
            return 0;
          }
          if (header.block_ != last_block_ + 1)
          {
            ROS_DEBUG("Expected block %d, received %d", last_block_ + 1, header.block_);
            data_filled_ = 0; // discard datagram
            return 0;
          }
          last_block_ = header.block_;

          break;
        default:
          ROS_ERROR("Unexpected UDP header OP [%d]", header.op_);
          return -1;
      }

      bytes_read += copy_bytes;

      if (last_block_ == (total_blocks_ - 1))
      {
        current_message_id_ = 0;
        break;
      }
    }
  }

  return bytes_read;
}

int32_t TransportUDSDatagram::write(uint8_t* buffer, uint32_t size)
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to write on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT((int32_t)size > 0);

  const uint32_t max_payload_size = max_datagram_size_ - sizeof(TransportUDSDatagramHeader);

  uint32_t bytes_sent = 0;
  uint32_t this_block = 0;
  if (++current_message_id_ == 0)
    ++current_message_id_;
  while (bytes_sent < size)
  {
    TransportUDSDatagramHeader header;
    header.connection_id_ = connection_id_;
    header.message_id_ = current_message_id_;
    if (this_block == 0)
    {
      header.op_ = ROS_UDP_DATA0;
      header.block_ = (size + max_payload_size - 1) / max_payload_size;
    }
    else
    {
      header.op_ = ROS_UDP_DATAN;
      header.block_ = this_block;
    }
    ++this_block;

    struct iovec iov[2];
    iov[0].iov_base = &header;
    iov[0].iov_len = sizeof(header);
    iov[1].iov_base = buffer + bytes_sent;
    iov[1].iov_len = std::min(max_payload_size, size - bytes_sent);
    ssize_t num_bytes = writev(sock_, iov, 2);

    if (num_bytes < 0)
    {
      if( !last_socket_error_is_would_block() ) // Actually EAGAIN or EWOULDBLOCK on posix
      {
        ROSCPP_LOG_DEBUG("writev() failed with error [%s]", last_socket_error_string());
        close();
        break;
      }
      else
      {
        num_bytes = 0;
        --this_block;
      }
    }
    else if (num_bytes < (unsigned) sizeof(header))
    {
      ROSCPP_LOG_DEBUG("Socket [%d] short write (%d bytes), closing", sock_, int(num_bytes));
      close();
      break;
    }
    else
    {
      num_bytes -= sizeof(header);
    }
    bytes_sent += num_bytes;
  }

  return bytes_sent;
}

void TransportUDSDatagram::enableRead()
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

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

void TransportUDSDatagram::disableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::mutex::scoped_lock lock(close_mutex_);

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

void TransportUDSDatagram::enableWrite()
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

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

void TransportUDSDatagram::disableWrite()
{
  {
    boost::mutex::scoped_lock lock(close_mutex_);

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

TransportUDSDatagramPtr TransportUDSDatagram::createOutgoing(const std::string& uds_path, int connection_id, int max_datagram_size)
{
  ROS_ASSERT(is_server_);

  TransportUDSDatagramPtr transport(boost::make_shared<TransportUDSDatagram>(poll_set_, flags_, max_datagram_size));
  if (!transport->connect(uds_path, connection_id))
  {
    ROS_ERROR("Failed to create outgoing connection");
    return TransportUDSDatagramPtr();
  }
  return transport;
}

std::string TransportUDSDatagram::getClientURI()
{
  ROS_ASSERT(!is_server_);

  std::stringstream uri;
  uri << "localhost(Unix Domain Socket)";
  return uri.str();
}

}

#endif // ROS_UDS_EXT_DISABLE
