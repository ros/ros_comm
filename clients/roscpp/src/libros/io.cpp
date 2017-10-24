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
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/io.h>
#include <ros/assert.h> // don't need if we dont call the pipe functions.
#include <ros/file_log.h>
#include <errno.h> // for EFAULT and co.
#include <iostream>
#include <sstream>
#ifdef WIN32
#else
  #include <cstring> // strerror
  #include <fcntl.h> // for non-blocking configuration
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros {

int last_socket_error() {
	#ifdef WIN32
		return WSAGetLastError();
	#else
		return errno;
	#endif
}

const char* socket_error_string(int err) {
#ifdef WIN32
    // could fix this to use FORMAT_MESSAGE and print a real string later,
    // but not high priority.
    std::stringstream ostream;
    ostream << "WSA Error: " << err;
    return ostream.str().c_str();
#else
    return strerror(err);
#endif
}

const char* last_socket_error_string() {
	#ifdef WIN32
        return socket_error_string(WSAGetLastError());
	#else
        return socket_error_string(errno);
	#endif
}

bool last_socket_error_is_would_block() {
#if defined(WIN32)
	if ( WSAGetLastError() == WSAEWOULDBLOCK ) {
		return true;
	} else {
		return false;
	}
#else
	if ( ( errno == EAGAIN  ) || ( errno == EWOULDBLOCK ) ) { // posix permits either
		return true;
	} else {
		return false;
	}
#endif
}

/*****************************************************************************
** Service Robotics/Libssh Functions
*****************************************************************************/
/**
 * @brief A cross platform polling function for sockets.
 *
 * Windows doesn't have a polling function until Vista (WSAPoll) and even then
 * its implementation is not supposed to be great. This works for a broader
 * range of platforms and will suffice.
 * @param fds - the socket set (descriptor's and events) to poll for.
 * @param nfds - the number of descriptors to poll for.
 * @param timeout - timeout in milliseconds.
 * @return int : -1 on error, 0 on timeout, +ve number of structures with received events.
 */
int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout) {
#if defined(WIN32)
	fd_set readfds, writefds, exceptfds;
	struct timeval tv, *ptv;
	socket_fd_t max_fd;
	int rc;
	nfds_t i;

	if (fds == NULL) {
		errno = EFAULT;
		return -1;
	}

	FD_ZERO (&readfds);
	FD_ZERO (&writefds);
	FD_ZERO (&exceptfds);

	/*********************
	** Compute fd sets
	**********************/
	// also find the largest descriptor.
	for (rc = -1, max_fd = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd == INVALID_SOCKET) {
			continue;
		}
		if (fds[i].events & (POLLIN | POLLRDNORM)) {
			FD_SET (fds[i].fd, &readfds);
		}
		if (fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND)) {
			FD_SET (fds[i].fd, &writefds);
		}
		if (fds[i].events & (POLLPRI | POLLRDBAND)) {
			FD_SET (fds[i].fd, &exceptfds);
		}
		if (fds[i].fd > max_fd &&
			  (fds[i].events & (POLLIN | POLLOUT | POLLPRI |
								POLLRDNORM | POLLRDBAND |
								POLLWRNORM | POLLWRBAND))) {
			max_fd = fds[i].fd;
			rc = 0;
		}
	}

	if (rc == -1) {
		errno = EINVAL;
		return -1;
	}
	/*********************
	** Setting the timeout
	**********************/
	if (timeout < 0) {
		ptv = NULL;
	} else {
		ptv = &tv;
		if (timeout == 0) {
			tv.tv_sec = 0;
			tv.tv_usec = 0;
		} else {
			tv.tv_sec = timeout / 1000;
			tv.tv_usec = (timeout % 1000) * 1000;
		}
	}

	rc = select (max_fd + 1, &readfds, &writefds, &exceptfds, ptv);
	if (rc < 0) {
		return -1;
	} else if ( rc == 0 ) {
		return 0;
	}

	for (rc = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd != INVALID_SOCKET) {
			fds[i].revents = 0;

			if (FD_ISSET(fds[i].fd, &readfds)) {
				int save_errno = errno;
				char data[64] = {0};
				int ret;

				/* support for POLLHUP */
				// just check if there's incoming data, without removing it from the queue.
				ret = recv(fds[i].fd, data, 64, MSG_PEEK);
				#ifdef WIN32
				if ((ret == -1) &&
						(errno == WSAESHUTDOWN || errno == WSAECONNRESET ||
						(errno == WSAECONNABORTED) || errno == WSAENETRESET))
				#else
				if ((ret == -1) &&
						(errno == ESHUTDOWN || errno == ECONNRESET ||
						(errno == ECONNABORTED) || errno == ENETRESET))
				#endif
				{
					fds[i].revents |= POLLHUP;
				} else {
					fds[i].revents |= fds[i].events & (POLLIN | POLLRDNORM);
				}
				errno = save_errno;
			}
			if (FD_ISSET(fds[i].fd, &writefds)) {
				fds[i].revents |= fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND);
			}

			if (FD_ISSET(fds[i].fd, &exceptfds)) {
				fds[i].revents |= fds[i].events & (POLLPRI | POLLRDBAND);
			}

			if (fds[i].revents & ~POLLHUP) {
				rc++;
			}
		} else {
				fds[i].revents = POLLNVAL;
		}
	}
	return rc;
#else
	// use an existing poll implementation
	int result = poll(fds, nfds, timeout);
	if ( result < 0 ) {
		// EINTR means that we got interrupted by a signal, and is not an error
		if(errno == EINTR) {
			result = 0;
		}
	}
	return result;
#endif // poll_sockets functions
}
/*****************************************************************************
** Socket Utilities
*****************************************************************************/
/**
 * Checks if the async socket connection is success, failure or connecting.
 * @param err - WSAGetLastError()/errno on failure.
 * @return int : 1 on connected, 0 on connecting, -1 on failure.
 */
int is_async_connected(socket_fd_t &socket, int &err) {
    // use zero-timeout select to check if async socket
    fd_set wfds, exceptfds;
    FD_ZERO(&wfds);
    FD_ZERO(&exceptfds);
    FD_SET(socket, &wfds);
    FD_SET(socket, &exceptfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int ret = select(socket + 1, NULL, &wfds, &exceptfds, &tv);
    if (ret == -1)
    {
      // select() error
      ROSCPP_CONN_LOG_DEBUG("select() on socket[%d] failed with error [%s]",
                            socket, last_socket_error_string());
      err = last_socket_error();
      return -1;
    }
    else if (ret == 0)
    {
      // select() timeout, socket is still connecting
      err = 0;
      return 0;
    }
    // select() fake wakeup?
    ROS_ASSERT(FD_ISSET(socket, &wfds) || FD_ISSET(socket, &exceptfds));

    // check connection is success or failure
#ifdef WIN32
    // In Windows:
    //    Success is reported in the writefds set and failure is reported in the exceptfds set.
    //    If failure, we must then call getsockopt SO_ERROR to determine the error value to describe why the failure occurred
    ROS_ASSERT((FD_ISSET(socket, &wfds) && !FD_ISSET(socket, &exceptfds)) ||
               (!FD_ISSET(socket, &wfds) && FD_ISSET(socket, &exceptfds)));
    if (FD_ISSET(socket, &exceptfds)) {
      // an error occurred during connection
      int errinfo;
      socklen_t errlen = sizeof(int);
      if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &errinfo, &errlen) == -1)
      {
        // getsockopt() error
        ROSCPP_CONN_LOG_DEBUG("getsockopt() on socket[%d] failed with error [%s]",
                              socket, last_socket_error_string());
        err = last_socket_error();
        return -1;
      }
      ROS_ASSERT(errlen == sizeof(int));
      ROS_ASSERT(errinfo != 0);
      ROSCPP_CONN_LOG_DEBUG("Async connect on socket[%d] failed with error [%s]",
                            socket, socket_error_string(errinfo));
      err = errinfo;
      return -1;
    }
#else
    // In Linux:
    //    Both success and failure is reported in the writefds set.
    //    We must use getsockopt() to read the SO_ERROR option at level SOL_SOCKET to determine whether
    //    connect() completed successfully (SO_ERROR is zero) or unsuccessfully (SO_ERROR is error code).
    if (!FD_ISSET(socket, &wfds)) {
      // wfds is not set, socket is still connecting
      err = 0;
      return 0;
    }
    // use getsockopt() to check connection is success or failure
    int errinfo;
    socklen_t errlen = sizeof(int);
    if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &errinfo, &errlen) == -1)
    {
      // getsockopt() error
      ROSCPP_CONN_LOG_DEBUG("getsockopt() on socket[%d] failed with error [%s]",
                            socket, last_socket_error_string());
      err = last_socket_error();
      return -1;
    }
    ROS_ASSERT(errlen == sizeof(int));
    if (errinfo != 0)
    {
      // an error occurred during connection
      ROSCPP_CONN_LOG_DEBUG("Async connect on socket[%d] failed with error [%s]",
                            socket, socket_error_string(errinfo));
      err = errinfo;
      return -1;
    }
#endif
    err = 0;
    return 1;
}

/**
 * Sets the socket as non blocking.
 * @return int : 0 on success, WSAGetLastError()/errno on failure.
 */
int set_non_blocking(socket_fd_t &socket) {
#ifdef WIN32
    u_long non_blocking = 1;
	if(ioctlsocket( socket, FIONBIO, &non_blocking ) != 0 )
	{
      return WSAGetLastError();
	}
#else
    if(fcntl(socket, F_SETFL, O_NONBLOCK) == -1)
    {
      return errno;
    }
#endif
    return 0;
}

/**
 * @brief Close the socket.
 *
 * @return int : 0 on success, -1 on failure.
 */
int close_socket(socket_fd_t &socket) {
#ifdef WIN32
	if(::closesocket(socket) == SOCKET_ERROR ) {
		return -1;
	} else {
		return 0;
	}
#else
	if (::close(socket) < 0) {
		return -1;
	} else {
		return 0;
	}
#endif //WIN32
}

/*****************************************************************************
** Signal Pair
*****************************************************************************/
/**
 * This code is primarily from the msdn socket tutorials.
 * @param signal_pair : a pair of sockets linked to each other over localhost.
 * @return 0 on success, -1 on failure.
 */
int create_signal_pair(signal_fd_t signal_pair[2]) {
#ifdef WIN32 // use a socket pair
	signal_pair[0] = INVALID_SOCKET;
	signal_pair[1] = INVALID_SOCKET;

    union {
       struct sockaddr_in inaddr;
       struct sockaddr addr;
    } a;
    socklen_t addrlen = sizeof(a.inaddr);

    /*********************
	** Listen Socket
	**********************/
	socket_fd_t listen_socket = INVALID_SOCKET;
    listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listen_socket == INVALID_SOCKET) {
		return -1;
	}

    // allow it to be bound to an address already in use - do we actually need this?
    int reuse = 1;
    if (setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, (char*) &reuse, (socklen_t) sizeof(reuse)) == SOCKET_ERROR ) {
    	::closesocket(listen_socket);
		return -1;
    }

    memset(&a, 0, sizeof(a));
    a.inaddr.sin_family = AF_INET;
    a.inaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // For TCP/IP, if the port is specified as zero, the service provider assigns
    // a unique port to the application from the dynamic client port range.
    a.inaddr.sin_port = 0;

    if  (bind(listen_socket, &a.addr, sizeof(a.inaddr)) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return -1;
    }
    // we need this below because the system auto filled in some entries, e.g. port #
    if  (getsockname(listen_socket, &a.addr, &addrlen) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return -1;
    }
    // max 1 connection permitted
    if (listen(listen_socket, 1) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return -1;
    }

    /*********************
	** Connection
	**********************/
    // do we need io overlapping?
    // DWORD flags = (make_overlapped ? WSA_FLAG_OVERLAPPED : 0);
    DWORD overlapped_flag = 0;
    signal_pair[0] = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, overlapped_flag);
    if (signal_pair[0] == INVALID_SOCKET) {
    	::closesocket(listen_socket);
    	::closesocket(signal_pair[0]);
    	return -1;
    }
    // reusing the information from above to connect to the listener
    if (connect(signal_pair[0], &a.addr, sizeof(a.inaddr)) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
    	::closesocket(signal_pair[0]);
    	return -1;
    }
    /*********************
	** Accept
	**********************/
    signal_pair[1] = accept(listen_socket, NULL, NULL);
    if (signal_pair[1] == INVALID_SOCKET) {
    	::closesocket(listen_socket);
    	::closesocket(signal_pair[0]);
    	::closesocket(signal_pair[1]);
    	return -1;
    }
	/*********************
	** Nonblocking
	**********************/
    // should we do this or should we set io overlapping?
    if ( (set_non_blocking(signal_pair[0]) != 0) || (set_non_blocking(signal_pair[1]) != 0)  ) {
		::closesocket(listen_socket);
		::closesocket(signal_pair[0]);
		::closesocket(signal_pair[1]);
    	return -1;
    }
	/*********************
	** Cleanup
	**********************/
    ::closesocket(listen_socket);  // the listener has done its job.
    return 0;
#else // use a pipe pair
	// initialize
	signal_pair[0] = -1;
	signal_pair[1] = -1;

	if(pipe(signal_pair) != 0) {
		ROS_FATAL( "pipe() failed");
		return -1;
	}
	if(fcntl(signal_pair[0], F_SETFL, O_NONBLOCK) == -1) {
		ROS_FATAL( "fcntl() failed");
		return -1;
	}
	if(fcntl(signal_pair[1], F_SETFL, O_NONBLOCK) == -1) {
		ROS_FATAL( "fcntl() failed");
		return -1;
	}
	return 0;
#endif // create_pipe
}

} // namespace ros
