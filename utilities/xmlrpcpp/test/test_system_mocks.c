
#define MOCK_SYSCALL(ret, name, ARG_TYPES, ARG_NAMES)                          \
  ret __real_##name ARG_TYPES;                                                 \
  ret(*fake_##name) ARG_TYPES = 0;                                             \
  ret __wrap_##name ARG_TYPES {                                                \
    if (fake_##name) {                                                         \
      return fake_##name ARG_NAMES;                                            \
    } else {                                                                   \
      return __real_##name ARG_NAMES;                                          \
    }                                                                          \
  }                                                                            \
  int name##_calls = 0;                                                        \
  ret count_##name ARG_TYPES {                                                 \
    name##_calls++;                                                            \
    return 0;                                                                  \
  }

#include "test_system_mocks.h"

// custom mock for fcntl because it is varargs
// the mocked version always takes the third argument
int __real_fcntl(int fd, int cmd, ...);
int (*fake_fcntl)(int fd, int cmd, unsigned long) = 0;
int __wrap_fcntl(int fd, int cmd, ...) {
  va_list ap;
  va_start(ap, cmd);
  unsigned long arg = va_arg(ap, unsigned long);
  va_end(ap);

  if (fake_fcntl) {
    return fake_fcntl(fd, cmd, arg);
  } else {
    return __real_fcntl(fd, cmd, arg);
  }
}
int fcntl_calls = 0;
int count_fcntl(int fd, int cmd, unsigned long arg) {
  fcntl_calls++;
  return 0;
}

// Custom mock for freeaddrinfo because it returns void.
void __real_freeaddrinfo(struct addrinfo* res);
void (*fake_freeaddrinfo)(struct addrinfo* res) = 0;
void __wrap_freeaddrinfo(struct addrinfo* res) {
  if (fake_freeaddrinfo) {
    return fake_freeaddrinfo(res);
  } else {
    return __real_freeaddrinfo(res);
  }
}
int freeaddrinfo_calls = 0;
void count_freeaddrinfo(struct addrinfo* res) {
  freeaddrinfo_calls++;
  return;
}
