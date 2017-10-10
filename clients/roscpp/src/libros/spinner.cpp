/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/spinner.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <tracetools/tracetools.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace {

const std::string DEFAULT_ERROR_MESSAGE =
    "\nAttempt to spin a callback queue from two spinners, one of them being single-threaded."
    "\nThis will probably result in callbacks being executed out-of-order."
    "\nIn future this will throw an exception!";

/** class to monitor running single-threaded spinners.
 *
 *  Calling the callbacks of a callback queue _in order_, requires a unique SingleThreadedSpinner
 *  spinning on the queue. Other threads accessing the callback queue will probably intercept execution order.

 *  To avoid multiple SingleThreadedSpinners (started from different threads) to operate on the same callback queue,
 *  this class stores a map of all spinned callback queues.
 *  If the spinner is single threaded, the corresponding thread-id is stored in the map
 *  and if other threads will try to spin the same queue, an error message is issued.
 *
 *  If the spinner is multi-threaded, the stored thread-id is NULL and future SingleThreadedSpinners
 *  should not spin this queue. However, other multi-threaded spinners are allowed.
 */
struct SpinnerMonitor
{
  /* store spinner information per callback queue:
     Only alike spinners (single-threaded or multi-threaded) are allowed on a callback queue.
     For single-threaded spinners we store their thread id.
     We store the number of alike spinners operating on the callback queue.
  */
  struct Entry
  {
    Entry(const boost::thread::id &tid,
          const boost::thread::id &initial_tid) : tid(tid), initial_tid(initial_tid), num(0) {}

    boost::thread::id tid; // proper thread id of single-threaded spinner
    boost::thread::id initial_tid; // to retain old behaviour, store first spinner's thread id
    unsigned int num; // number of (alike) spinners serving this queue
  };

  /// add a queue to the list
  bool add(ros::CallbackQueue* queue, bool single_threaded)
  {
    boost::mutex::scoped_lock lock(mutex_);

    boost::thread::id current_tid = boost::this_thread::get_id();
    boost::thread::id tid; // current thread id for single-threaded spinners, zero for multi-threaded ones
    if (single_threaded)
      tid = current_tid;

    std::map<ros::CallbackQueue*, Entry>::iterator it = spinning_queues_.find(queue);
    bool can_spin = ( it == spinning_queues_.end() || // we will spin on any new queue
                      it->second.tid == tid ); // otherwise spinner must be alike (all multi-threaded: 0, or single-threaded on same thread id)

    if (!can_spin)
    {
      // Previous behavior (up to Kinetic) was to accept multiple spinners on a queue
      // as long as they were started from the same thread. Although this is wrong behavior,
      // we retain it here for backwards compatibility, i.e. we allow spinning of a
      // single-threaded spinner after several multi-threaded ones, given that they
      // were started from the same initial thread
      if (it->second.initial_tid == tid)
      {
        ROS_ERROR_STREAM("SpinnerMonitor: single-threaded spinner after multi-threaded one(s)."
                         << DEFAULT_ERROR_MESSAGE
                         << " Only allowed for backwards compatibility.");
        it->second.tid = tid; // "upgrade" tid to represent single-threaded spinner
      }
      else
        return false;
    }

    if (it == spinning_queues_.end())
      it = spinning_queues_.insert(it, std::make_pair(queue, Entry(tid, current_tid)));

    // increment number of active spinners
    it->second.num += 1;

    return true;
  }

  /// remove a queue from the list
  void remove(ros::CallbackQueue* queue)
  {
    boost::mutex::scoped_lock lock(mutex_);
    std::map<ros::CallbackQueue*, Entry>::iterator it = spinning_queues_.find(queue);
    ROS_ASSERT_MSG(it != spinning_queues_.end(), "Call to SpinnerMonitor::remove() without matching call to add().");

    if (it->second.tid != boost::thread::id() && it->second.tid != boost::this_thread::get_id())
    {
      // This doesn't harm, but isn't good practice?
      // It was enforced by the previous implementation.
      ROS_WARN("SpinnerMonitor::remove() called from different thread than add().");
    }

    ROS_ASSERT_MSG(it->second.num > 0, "SpinnerMonitor::remove(): Invalid spinner count (0) encountered.");
    it->second.num -= 1;
    if (it->second.num == 0)
      spinning_queues_.erase(it); // erase queue entry to allow future queues with same pointer
  }

  std::map<ros::CallbackQueue*, Entry> spinning_queues_;
  boost::mutex mutex_;
};

SpinnerMonitor spinner_monitor;
}

namespace ros
{


void SingleThreadedSpinner::spin(CallbackQueue* queue)
{
  ros::trace::task_init("SingleThreadedSpinner::spin");

  if (!queue)
  {
    queue = getGlobalCallbackQueue();
  }

  if (!spinner_monitor.add(queue, true))
  {
    ROS_ERROR_STREAM("SingleThreadedSpinner: " << DEFAULT_ERROR_MESSAGE);
    return;
  }

  ros::WallDuration timeout(0.1f);
  ros::NodeHandle n;
  while (n.ok())
  {
    queue->callAvailable(timeout);
  }
  spinner_monitor.remove(queue);
}

MultiThreadedSpinner::MultiThreadedSpinner(uint32_t thread_count)
: thread_count_(thread_count)
{
}

void MultiThreadedSpinner::spin(CallbackQueue* queue)
{
  AsyncSpinner s(thread_count_, queue);
  s.start();

  ros::waitForShutdown();
}

class AsyncSpinnerImpl
{
public:
  AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue);
  ~AsyncSpinnerImpl();

  bool canStart();
  void start();
  void stop();

private:
  void threadFunc();

  boost::mutex mutex_;
  boost::thread_group threads_;

  uint32_t thread_count_;
  CallbackQueue* callback_queue_;

  volatile bool continue_;

  ros::NodeHandle nh_;
};

AsyncSpinnerImpl::AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue)
: thread_count_(thread_count)
, callback_queue_(queue)
, continue_(false)
{
  if (thread_count == 0)
  {
    thread_count_ = boost::thread::hardware_concurrency();

    if (thread_count_ == 0)
    {
      thread_count_ = 1;
    }
  }

  if (!queue)
  {
    callback_queue_ = getGlobalCallbackQueue();
  }
}

AsyncSpinnerImpl::~AsyncSpinnerImpl()
{
  stop();
}

bool AsyncSpinnerImpl::canStart()
{
  return true;
}

void AsyncSpinnerImpl::start()
{
  boost::mutex::scoped_lock lock(mutex_);

  if (continue_)
    return; // already spinning

  if (!spinner_monitor.add(callback_queue_, false))
  {
    ROS_ERROR_STREAM("AsyncSpinnerImpl: " << DEFAULT_ERROR_MESSAGE);
    return;
  }

  continue_ = true;

  for (uint32_t i = 0; i < thread_count_; ++i)
  {
    threads_.create_thread(boost::bind(&AsyncSpinnerImpl::threadFunc, this));
  }
}

void AsyncSpinnerImpl::stop()
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!continue_)
    return;

  continue_ = false;
  threads_.join_all();

  spinner_monitor.remove(callback_queue_);
}

void AsyncSpinnerImpl::threadFunc()
{
  ros::trace::task_init("AsyncSpinner::threadFunc");

  disableAllSignalsInThisThread();

  CallbackQueue* queue = callback_queue_;
  bool use_call_available = thread_count_ == 1;
  WallDuration timeout(0.1);

  while (continue_ && nh_.ok())
  {
    if (use_call_available)
    {
      queue->callAvailable(timeout);
    }
    else
    {
      queue->callOne(timeout);
    }
  }
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count)
: impl_(new AsyncSpinnerImpl(thread_count, 0))
{
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count, CallbackQueue* queue)
: impl_(new AsyncSpinnerImpl(thread_count, queue))
{
}

bool AsyncSpinner::canStart()
{
  return impl_->canStart();
}

void AsyncSpinner::start()
{
  impl_->start();
}

void AsyncSpinner::stop()
{
  impl_->stop();
}

}
