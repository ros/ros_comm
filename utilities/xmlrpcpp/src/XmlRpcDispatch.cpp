
#include "xmlrpcpp/XmlRpcDispatch.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcUtil.h"

#include <math.h>
#include <errno.h>
#include <sys/timeb.h>
#include <sys/poll.h>
#if defined (__ANDROID__)
#include <sys/select.h>
#endif

#if defined(_WINDOWS)
# include <winsock2.h>
static inline int poll( struct pollfd *pfd, int nfds, int timeout)
{
    return WSAPoll ( pfd, nfds, timeout );
}

# define USE_FTIME
# if defined(_MSC_VER)
#  define timeb _timeb
#  define ftime _ftime_s
# endif
#else
# include <sys/time.h>
#endif  // _WINDOWS


using namespace XmlRpc;


XmlRpcDispatch::XmlRpcDispatch()
{
  _endTime = -1.0;
  _doClear = false;
  _inWork = false;
}


XmlRpcDispatch::~XmlRpcDispatch()
{
}

// Monitor this source for the specified events and call its event handler
// when the event occurs
void
XmlRpcDispatch::addSource(XmlRpcSource* source, unsigned mask)
{
  _sources.push_back(MonitoredSource(source, mask));
}

// Stop monitoring this source. Does not close the source.
void
XmlRpcDispatch::removeSource(XmlRpcSource* source)
{
  for (SourceList::iterator it=_sources.begin(); it!=_sources.end(); ++it)
    if (it->getSource() == source)
    {
      _sources.erase(it);
      break;
    }
}


// Modify the types of events to watch for on this source
void
XmlRpcDispatch::setSourceEvents(XmlRpcSource* source, unsigned eventMask)
{
  for (SourceList::iterator it=_sources.begin(); it!=_sources.end(); ++it)
    if (it->getSource() == source)
    {
      it->getMask() = eventMask;
      break;
    }
}



// Watch current set of sources and process events
void
XmlRpcDispatch::work(double timeout)
{
  // Compute end time
  _endTime = (timeout < 0.0) ? -1.0 : (getTime() + timeout);
  _doClear = false;
  _inWork = true;
  int timeout_ms = static_cast<int>(floor(timeout * 1000.));

  // Only work while there is something to monitor
  while (_sources.size() > 0) {

    // Construct the sets of descriptors we are interested in
    struct pollfd fds[_sources.size()];

    SourceList::iterator it;
    std::size_t i = 0;
    for (it=_sources.begin(); it!=_sources.end(); ++it, ++i) {
      int fd = it->getSource()->getfd();
      fds[i].fd = fd;
      fds[i].events = 0;
      if (it->getMask() & ReadableEvent) fds[i].events |= POLLIN;
      if (it->getMask() & WritableEvent) fds[i].events |= POLLOUT;
    }

    // Check for events
    int nEvents = poll(fds, _sources.size(), (timeout_ms < 0) ? -1 : timeout_ms);

    if (nEvents < 0)
    {
      if(errno != EINTR)
        XmlRpcUtil::error("Error in XmlRpcDispatch::work: error in poll (%d).", nEvents);
      _inWork = false;
      return;
    }

    // Process events
    for (i=0, it=_sources.begin(); it != _sources.end(); ++i)
    {
      SourceList::iterator thisIt = it++;
      XmlRpcSource* src = thisIt->getSource();
      unsigned newMask = (unsigned) -1;
      // If you select on multiple event types this could be ambiguous
      if (fds[i].revents & POLLIN)
        newMask &= src->handleEvent(ReadableEvent);
      if (fds[i].revents & POLLOUT)
        newMask &= src->handleEvent(WritableEvent);
      if (fds[i].revents & (POLLERR|POLLHUP|POLLNVAL))
        newMask &= src->handleEvent(Exception);

      // Find the source again.  It may have moved as a result of the way
      // that sources are removed and added in the call stack starting
      // from the handleEvent() calls above.
      for (thisIt=_sources.begin(); thisIt != _sources.end(); thisIt++)
      {
        if(thisIt->getSource() == src)
          break;
      }
      if(thisIt == _sources.end())
      {
        XmlRpcUtil::error("Error in XmlRpcDispatch::work: couldn't find source iterator");
        continue;
      }

      if ( ! newMask) {
        _sources.erase(thisIt);  // Stop monitoring this one
        if ( ! src->getKeepOpen())
          src->close();
      } else if (newMask != (unsigned) -1)
        thisIt->getMask() = newMask;
    }

    // Check whether to clear all sources
    if (_doClear)
    {
      SourceList closeList = _sources;
      _sources.clear();
      for (SourceList::iterator it=closeList.begin(); it!=closeList.end(); ++it) {
	XmlRpcSource *src = it->getSource();
        src->close();
      }

      _doClear = false;
    }

    // Check whether end time has passed
    if (0 <= _endTime && getTime() > _endTime)
      break;
  }

  _inWork = false;
}


// Exit from work routine. Presumably this will be called from
// one of the source event handlers.
void
XmlRpcDispatch::exit()
{
  _endTime = 0.0;   // Return from work asap
}

// Clear all sources from the monitored sources list
void
XmlRpcDispatch::clear()
{
  if (_inWork)
    _doClear = true;  // Finish reporting current events before clearing
  else
  {
    SourceList closeList = _sources;
    _sources.clear();
    for (SourceList::iterator it=closeList.begin(); it!=closeList.end(); ++it)
      it->getSource()->close();
  }
}


double
XmlRpcDispatch::getTime()
{
#ifdef USE_FTIME
  struct timeb	tbuff;

  ftime(&tbuff);
  return ((double) tbuff.time + ((double)tbuff.millitm / 1000.0) +
	  ((double) tbuff.timezone * 60));
#else
  struct timeval	tv;
  struct timezone	tz;

  gettimeofday(&tv, &tz);
  return (tv.tv_sec + tv.tv_usec / 1000000.0);
#endif /* USE_FTIME */
}


