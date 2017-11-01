#include "xmlrpcpp/XmlRpcDispatch.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcSocket.h"
#include "mock_socket.h"

#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include <iostream>
#include <functional>

#include <gtest/gtest.h>

// Mocks for select and poll. The build file specifies -Wl,--wrap for both of
// these, so the original symbols are available as __real_xxx and any uses of
// those symbols instead use __wrap_xxx
extern "C" {
int __real_select(int nfds,
    fd_set* readfds,
    fd_set* writefds,
    fd_set* exceptfds,
    struct timeval* timeout);

int (*fake_select)(int nfds,
                   fd_set* readfds,
                   fd_set* writefds,
                   fd_set* exceptfds,
                   struct timeval* timeout) = 0;

// Intercept hook for select that executes fake_select if it is set, and
// otherwise calls the actual __select
int __wrap_select(int nfds,
           fd_set* readfds,
           fd_set* writefds,
           fd_set* exceptfds,
           struct timeval* timeout) {
  if (fake_select) {
    return fake_select(nfds, readfds, writefds, exceptfds, timeout);
  } else {
    return __real_select(nfds, readfds, writefds, exceptfds, timeout);
  }
}

// Mock for poll
int __real_poll(struct pollfd *fds, nfds_t nfds, int timeout);

int (*fake_poll)(struct pollfd *, nfds_t, int) = 0;

int __wrap_poll(struct pollfd *fds, nfds_t nfds, int timeout) {
  if(fake_poll) {
    return fake_poll(fds, nfds, timeout);
  } else {
    return __real_poll(fds, nfds, timeout);
  }
}

}

// Timeval comparator so that EXPECT_EQ works for timevals.
bool operator==(struct timeval t1, struct timeval t2) {
  return t1.tv_sec == t2.tv_sec && t1.tv_usec == t2.tv_usec;
}

// gtest print operator for timeval so that we get good messages if a timeval
// comparison fails.
void PrintTo(struct timeval tv, std::ostream *os) {
  *os << "{ .tv_sec = " << tv.tv_sec << ", .tv_usec = " << tv.tv_usec << "}";
}

int select_calls = 0;
int select_ret = 0;
int select_errno = 0;
int select_nfds = 0;
struct timeval select_timeout;
std::set<int> select_readin;
std::set<int> select_writein;
std::set<int> select_exceptin;
std::set<int> select_readout;
std::set<int> select_writeout;
std::set<int> select_exceptout;
int mock_select(int nfds,
    fd_set* readfds,
    fd_set* writefds,
    fd_set* exceptfds,
    struct timeval* timeout) {
  EXPECT_EQ(1, select_calls);
  select_calls--;

  EXPECT_GE(FD_SETSIZE, nfds);
  EXPECT_EQ(select_nfds, nfds);

  // NOTE: This implementation of select() does not update the timeout, but
  // the linux implementation does. Both variants are POSIX-compliant.
  EXPECT_TRUE(NULL != timeout);
  if(NULL != timeout) {
    EXPECT_EQ(select_timeout, *timeout);
  }

  // Check that expected file descriptors are set or unset.
  for(int i=0; i<std::max(nfds, select_nfds); i++) {
    bool readin = select_readin.count(i) > 0;
    EXPECT_EQ(readin, FD_ISSET(i, readfds));

    bool writein = select_writein.count(i) > 0;
    EXPECT_EQ(writein, FD_ISSET(i, writefds));

    bool exceptin = select_exceptin.count(i) > 0;
    EXPECT_EQ(exceptin, FD_ISSET(i, exceptfds));
  }

  // Update sets with output data.
  FD_ZERO(readfds);
  FD_ZERO(writefds);
  FD_ZERO(exceptfds);
  for(int i=0; i<std::max(nfds, select_nfds); i++) {
    if( select_readout.count(i) > 0) {
      FD_SET(i, readfds);
    }
    if( select_writeout.count(i) > 0) {
      FD_SET(i, writefds);
    }
    if( select_exceptout.count(i) > 0) {
      FD_SET(i, exceptfds);
    }
  }

  // HACK: Sleep for the requested duration
  // This can be thought of as a simulation of select() getting an event
  // exactly at the end of the timeout window, but really it's just here
  // because the dispatch loop has its own timer to determine if it should call
  // select again, and we don't want multiple calls.
  if(NULL != timeout) {
    struct timespec ts = { .tv_sec = timeout->tv_sec, .tv_nsec = timeout->tv_usec*1000};
    // Call nanosleep repeatedly until it returns 0 (success).
    // On failure it will update ts with the remaining time to sleep.
    int ret = 0;
    do {
      ret = nanosleep(&ts, &ts);
    } while( ret != 0 && errno == EINTR);
  }

  // Output errno and return value.
  errno = select_errno;
  return select_ret;
}

void Expect_select(int nfds,
    std::set<int> readin, std::set<int> writein, std::set<int> exceptin,
    struct timeval timeout,
    std::set<int> readout, std::set<int> writeout, std::set<int> exceptout,
    int _errno, int ret) {
  EXPECT_EQ(0, select_calls) << "Test bug: Cannot expect more than one call to select";
  select_calls = 1;

  EXPECT_GE(FD_SETSIZE, nfds);
  select_nfds = nfds;
  select_timeout = timeout;

  select_readin = readin;
  select_writein = writein;
  select_exceptin = exceptin;
  select_readout = readout;
  select_writeout = writeout;
  select_exceptout = exceptout;

  select_errno = _errno;
  select_ret = ret;
}

int poll_calls = 0;
int poll_ret = 0;
int poll_errno = 0;
int poll_timeout = 0;
std::vector<pollfd> poll_fds;
int mock_poll(struct pollfd *fds, nfds_t nfds, int timeout) {
  EXPECT_EQ(1, poll_calls);
  poll_calls--;

  EXPECT_EQ(poll_fds.size(), nfds);
  EXPECT_EQ(poll_timeout, timeout);

  for(nfds_t i=0; i<std::min(nfds, poll_fds.size()); i++) {
    EXPECT_EQ(poll_fds[i].fd, fds[i].fd);
    EXPECT_EQ(poll_fds[i].events, fds[i].events);
    fds[i].revents = poll_fds[i].revents;
  }

  // HACK: Sleep for the requested duration
  // This can be thought of as a simulation of select() getting an event
  // exactly at the end of the timeout window, but really it's just here
  // because the dispatch loop has its own timer to determine if it should call
  // select again, and we don't want multiple calls.
  if(timeout > 0) {
    timespec ts;
    ts.tv_sec = timeout / 1000;
    ts.tv_nsec = (timeout % 1000) * 1000000;
    // Call nanosleep repeatedly until it returns 0 (success).
    // On failure it will update ts with the remaining time to sleep.
    int ret = 0;
    do {
      ret = nanosleep(&ts, &ts);
    } while( ret != 0 && errno == EINTR);
  }

  errno = poll_errno;
  return poll_ret;
}

void Expect_poll(std::vector<pollfd> fds, int timeout, int _errno, int ret) {
  EXPECT_EQ(0, poll_calls) << "Test bug: Cannot expect more than one call to poll";
  poll_calls = 1;
  poll_ret = ret;
  poll_errno = _errno;
  poll_timeout = timeout;
  poll_fds = fds;
}

using namespace XmlRpc;

class MockSource : public XmlRpcSource {
public:
  MockSource(int fd)
    : handleEvent_calls(0), last_event(0), event_result(0), close_calls(0) {
    setfd(fd);
  }

  virtual ~MockSource() {
  }

  virtual unsigned handleEvent(unsigned eventType) {
    handleEvent_calls++;
    last_event = eventType;
    return event_result;
  }

  virtual void close() {
    close_calls++;
  }

  // variables used for mocking
  int handleEvent_calls;
  unsigned last_event;
  unsigned event_result;

  int close_calls;
};

#define EXPECT_CLOSE_CALLS(n)                                                  \
  do {                                                                         \
    EXPECT_EQ(m.close_calls, n);                                               \
    m.close_calls = 0;                                                         \
  } while (0)

#define EXPECT_EVENTS(n)                                                       \
  do {                                                                         \
    EXPECT_EQ(m.handleEvent_calls, n);                                         \
    m.handleEvent_calls = 0;                                                   \
  } while (0)

#define EXPECT_EVENT(event)                                                    \
  do {                                                                         \
    EXPECT_EQ(m.last_event, event);                                            \
    EXPECT_EQ(m.handleEvent_calls, 1);                                         \
    m.handleEvent_calls = 0;                                                   \
  } while (0)

class MockSourceTest : public ::testing::Test {
  protected:
    MockSourceTest() : m(4) {
      tv.tv_sec = 0;
      tv.tv_usec = 100000;

      pollfd f = { .fd = 4, .events = 0, .revents = 0 };
      fds.push_back(f);
    }

    void SetUp() {
      fake_select = mock_select;
      select_calls = 0;

      fake_poll = mock_poll;
      poll_calls = 0;
    }

    void TearDown() {
      EXPECT_EQ(0, select_calls);
      fake_select = 0;
      select_calls = 0;

      EXPECT_EQ(0, poll_calls);
      fake_poll = 0;
      poll_calls = 0;
    }

    MockSource m;
    XmlRpcDispatch dispatch;
    struct timeval tv;
    std::vector<pollfd> fds;
};

/*
 * Use a socket to provide the requisite file descriptor
 *
 * Tests to perform on XmlRpcDispatch
 * - Matrix of the following options:
 *   - Proper handling of setKeepOpen
 *   - Proper handling of deleteOnClose
 *   - Proper handling of return values from handleEvent
 * - Proper handling of file descriptor states
 *   - Correct masking of events by the eventMask for the source
 *   - Correct handling of exceptional file descriptor states
 *     - These states seem to mostly be related to sending OOB data over TCP; I
 *       don't see a way to simulate them with pipes, but it should be possible
 *       to loop back a TCP connection and generate that condition directly
 *   - Check that the argument to handleEvent matches the event that was
 *     simulated
 *   - Check that handleEvent is not called if no event was triggered
 * - Proper handling of timeout in XmlRpcDispatch work() method
 * - Proper removal of sources from _sources when errors occur
 * - If possible, trigger error return values from select(); maybe one of the
 *   following error cases from the select(2) man page:
 *    - Invalid file descriptor in set (already closed?)
 *    - Signal caught
 *    - nfds negative or invalid timeout
 *    - unable to allocate memory
 * - Proper handling of multiple XmlRpcSource objects
 *   - Multiple events during a single work() cycle
 *   - Events delivered to the correct Source
 */

TEST_F(MockSourceTest, ReadEvent) {
  m.event_result = XmlRpcDispatch::ReadableEvent;
  dispatch.addSource(&m, XmlRpcDispatch::ReadableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1);

  // Select returns not readable; expect no events.
  fds[0].events = POLLIN;
  fds[0].revents = 0;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);

  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);

  // Select returns readable, expect readable event
  fds[0].events = POLLIN;
  fds[0].revents = POLLIN;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENT(XmlRpcDispatch::ReadableEvent);
}

TEST_F(MockSourceTest, WriteEvent) {
  m.setKeepOpen();
  m.event_result = 0;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1);

  // Select returns writeable, expect one write event.
  fds[0].events = POLLOUT;
  fds[0].revents = POLLOUT;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENT(XmlRpcDispatch::WritableEvent);
  // We have keepOpen set, so don't expect a close call
  EXPECT_CLOSE_CALLS(0);
  // However, even if keepOpen is set, we expect the socket to be removed from
  // the sources list
  EXPECT_EQ(dispatch._sources.size(), 0);

  // Expect no more events. Since there's nothing in the dispatch list, we
  // don't expect that select will be called.
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);
}

TEST_F(MockSourceTest, NonWriteable) {
  m.event_result = XmlRpcDispatch::WritableEvent;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1);

  // Select doesn't return writable.
  fds[0].events = POLLOUT;
  fds[0].revents = 0;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENTS(0);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EQ(dispatch._sources.size(), 1);
}

TEST_F(MockSourceTest, WriteClose) {
  m.event_result = 0;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1);

  // Socket is always writeable. Expect 1 write event since we clear the write
  // event flag after we write once
  fds[0].events = POLLOUT;
  fds[0].revents = POLLOUT;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENT(XmlRpcDispatch::WritableEvent);

  // Since we returned 0 from handleEvent and don't have keepOpen set, expect
  // that the dispatch has called close() once and that the size of sources is
  // now 0
  EXPECT_CLOSE_CALLS(1);
  EXPECT_EQ(dispatch._sources.size(), 0);

  // Expect no more events. Since there's nothing in the dispatch list, we
  // don't expect that select will be called.
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);
}

TEST_F(MockSourceTest, Exception) {
  m.event_result = XmlRpcDispatch::Exception;
  dispatch.addSource(&m, XmlRpcDispatch::Exception);
  EXPECT_EQ(dispatch._sources.size(), 1);

  // Select returns no exception, so expect that the handler was not called.
  fds[0].events = POLLPRI;
  fds[0].revents = 0;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENTS(0);

  // Make exception, expect exception event.
  fds[0].events = POLLPRI;
  fds[0].revents = POLLPRI;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EVENT(XmlRpcDispatch::Exception);
}

// Test that dispatch works (or doesn't) with file descriptors above 1024
TEST_F(MockSourceTest, LargeFd) {
  m.setfd(1025);
  m.event_result = XmlRpcDispatch::WritableEvent;
  dispatch.addSource(&m, XmlRpcDispatch::WritableEvent);
  EXPECT_EQ(dispatch._sources.size(), 1);

  // Make select return writable, expect 1 write event.
  fds[0].fd = 1025;
  fds[0].events = POLLOUT;
  fds[0].revents = POLLOUT;
  Expect_poll(fds, 100, 0, 0);
  dispatch.work(0.1);
  EXPECT_EVENT(XmlRpcDispatch::WritableEvent);
  EXPECT_CLOSE_CALLS(0);
  EXPECT_EQ(dispatch._sources.size(), 1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
