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

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace {

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
 *  should not spin this queue.
 */
struct SpinnerMonitor {
  /// add a queue to the list
  bool add(ros::CallbackQueue* queue, bool single_threaded)
  {
    boost::mutex::scoped_lock lock(mutex_);

    boost::thread::id tid;
    if (single_threaded)
      tid = boost::this_thread::get_id();

    std::map<ros::CallbackQueue*, boost::thread::id>::iterator it = spinning_queues_.find(queue);
    bool can_spin = ( it == spinning_queues_.end() || it->second == tid );
    if (!can_spin)
      return false;

    if (it == spinning_queues_.end())
      spinning_queues_.insert(it, std::make_pair(queue, tid));
    else if (single_threaded)
      it->second = tid; // "upgrade" thread id to "single-threaded"

    return true;
  }

  /// remove a queue from the list
  void remove(ros::CallbackQueue* queue)
  {
    boost::mutex::scoped_lock lock(mutex_);
    std::map<ros::CallbackQueue*, boost::thread::id>::const_iterator it = spinning_queues_.find(queue);
    if (it != spinning_queues_.end())
    {
      if (it->second != boost::thread::id() && it->second != boost::this_thread::get_id())
        ROS_ERROR("SpinnerMonitor::remove() called from different thread than add().");
      spinning_queues_.erase(it);
    }
  }

  std::map<ros::CallbackQueue*, boost::thread::id> spinning_queues_;
  boost::mutex mutex_;
};

SpinnerMonitor spinner_monitor;
const std::string DEFAULT_ERROR_MESSAGE =
    "Attempt to spin a callback queue from two spinners, one of them being single-threaded. "
    "This will probably result in callbacks being executed out-of-order.";
}

namespace ros
{


void SingleThreadedSpinner::spin(CallbackQueue* queue)
{
  if (!queue)
  {
    queue = getGlobalCallbackQueue();
  }

  if (!spinner_monitor.add(queue, true))
  {
    ROS_FATAL_STREAM("SingleThreadedSpinner: " << DEFAULT_ERROR_MESSAGE);
    throw std::runtime_error("There is already another spinner on this queue");
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
    ROS_FATAL_STREAM("AsyncSpinnerImpl: " << DEFAULT_ERROR_MESSAGE);
    throw std::runtime_error("There is already a single-threaded spinner on this queue");
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
