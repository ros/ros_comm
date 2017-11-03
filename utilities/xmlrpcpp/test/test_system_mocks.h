#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <limits.h>
#include <netdb.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef MOCK_SYSCALL
#define MOCK_SYSCALL(ret, name, ARG_TYPES, ARG_NAMES)                          \
  extern ret(*fake_##name) ARG_TYPES;                                          \
  extern int name##_calls;                                                     \
  ret count_##name ARG_TYPES;
#endif // MOCK_SYSCALL

#ifdef __cplusplus
extern "C" {
#endif

MOCK_SYSCALL(int,
             accept,
             (int sockfd, struct sockaddr* addr, socklen_t* addrlen),
             (sockfd, addr, addrlen));
MOCK_SYSCALL(int,
             bind,
             (int sockfd, const struct sockaddr* addr, socklen_t addrlen),
             (sockfd, addr, addrlen));
MOCK_SYSCALL(int, close, (int socket), (socket));
MOCK_SYSCALL(int,
             connect,
             (int sockfd, const struct sockaddr* addr, socklen_t addrlen),
             (sockfd, addr, addrlen));
MOCK_SYSCALL(int,
             getaddrinfo,
             (const char* node,
              const char* service,
              const struct addrinfo* hints,
              struct addrinfo** res),
             (node, service, hints, res));
MOCK_SYSCALL(int,
             getsockname,
             (int sockfd, struct sockaddr* addr, socklen_t* addrlen),
             (sockfd, addr, addrlen));
MOCK_SYSCALL(int, listen, (int sockfd, int backlog), (sockfd, backlog));
MOCK_SYSCALL(ssize_t,
             read,
             (int fd, void* buf, size_t count),
             (fd, buf, count));
MOCK_SYSCALL(
    int,
    setsockopt,
    (int sockfd, int level, int optname, const void* optval, socklen_t optlen),
    (sockfd, level, optname, optval, optlen));
MOCK_SYSCALL(int,
             select,
             (int nfds,
              fd_set* readfds,
              fd_set* writefds,
              fd_set* exceptfds,
              struct timeval* timeout),
             (nfds, readfds, writefds, exceptfds, timeout));
MOCK_SYSCALL(int,
             socket,
             (int domain, int type, int protocol),
             (domain, type, protocol));
MOCK_SYSCALL(ssize_t,
             write,
             (int fd, const void* buf, size_t count),
             (fd, buf, count));

// custom mock for fcntl because it is varargs
// the mocked version always takes the third argument
extern int (*fake_fcntl)(int fd, int cmd, unsigned long);
extern int fcntl_calls;
int count_fcntl(int fd, int cmd, unsigned long arg);

// Custom mock for freeaddrinfo because it returns void.
extern void (*fake_freeaddrinfo)(struct addrinfo* res);
extern int freeaddrinfo_calls;
void count_freeaddrinfo(struct addrinfo* res);

#ifdef __cplusplus
} // extern "C"
#endif
