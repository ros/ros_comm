/*
 * Copyright (C) 2017, Felix Ruess, Roboception GmbH
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

// Make sure we use CLOCK_MONOTONIC for the condition variable if not Apple.
#if !defined(__APPLE__) && !defined(WIN32)
#define BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC
#endif

#include "ros/steady_timer.h"
#include "ros/timer_manager.h"

// check if we have really included the backported boost condition variable
// just in case someone messes with the include order...
#if BOOST_VERSION < 106100
#ifndef USING_BACKPORTED_BOOST_CONDITION_VARIABLE
#error "steady timer needs boost version >= 1.61 or the backported headers!"
#endif
#endif

namespace ros
{

#if !defined(WIN32)
// specialization for SteadyTimer, to make sure we use a version with wait_until that uses the monotonic clock
template<>
void TimerManager<SteadyTime, WallDuration, SteadyTimerEvent>::threadFunc()
{
  SteadyTime current;
  while (!quit_)
  {
    SteadyTime sleep_end;

    boost::mutex::scoped_lock lock(timers_mutex_);

    current = SteadyTime::now();

    {
      boost::mutex::scoped_lock waitlock(waiting_mutex_);

      if (waiting_.empty())
      {
        sleep_end = current + WallDuration(0.1);
      }
      else
      {
        TimerInfoPtr info = findTimer(waiting_.front());

        while (!waiting_.empty() && info && info->next_expected <= current)
        {
          current = SteadyTime::now();

          //ROS_DEBUG("Scheduling timer callback for timer [%d] of period [%f], [%f] off expected", info->handle, info->period.toSec(), (current - info->next_expected).toSec());
          CallbackInterfacePtr cb(boost::make_shared<TimerQueueCallback>(this, info, info->last_expected, info->last_real, info->next_expected, info->last_expired, current));
          info->callback_queue->addCallback(cb, (uint64_t)info.get());

          waiting_.pop_front();

          if (waiting_.empty())
          {
            break;
          }

          info = findTimer(waiting_.front());
        }

        if (info)
        {
          sleep_end = info->next_expected;
        }
      }
    }

    while (!new_timer_ && SteadyTime::now() < sleep_end && !quit_)
    {
      current = SteadyTime::now();

      if (current >= sleep_end)
      {
        break;
      }

      // requires boost 1.61 for wait_until to actually use the steady clock
      // see: https://svn.boost.org/trac/boost/ticket/6377
      boost::chrono::steady_clock::time_point end_tp(boost::chrono::nanoseconds(sleep_end.toNSec()));
      timers_cond_.wait_until(lock, end_tp);
    }

    new_timer_ = false;
  }
}
#endif

SteadyTimer::Impl::Impl()
  : started_(false)
  , timer_handle_(-1)
{ }

SteadyTimer::Impl::~Impl()
{
  ROS_DEBUG("SteadyTimer deregistering callbacks.");
  stop();
}

void SteadyTimer::Impl::start()
{
  if (!started_)
  {
    VoidConstPtr tracked_object;
    if (has_tracked_object_)
    {
      tracked_object = tracked_object_.lock();
    }
    timer_handle_ = TimerManager<SteadyTime, WallDuration, SteadyTimerEvent>::global().add(period_, callback_, callback_queue_, tracked_object, oneshot_);
    started_ = true;
  }
}

void SteadyTimer::Impl::stop()
{
  if (started_)
  {
    started_ = false;
    TimerManager<SteadyTime, WallDuration, SteadyTimerEvent>::global().remove(timer_handle_);
    timer_handle_ = -1;
  }
}

bool SteadyTimer::Impl::isValid()
{
  return !period_.isZero();
}

bool SteadyTimer::Impl::hasPending()
{
  if (!isValid() || timer_handle_ == -1)
  {
    return false;
  }

  return TimerManager<SteadyTime, WallDuration, SteadyTimerEvent>::global().hasPending(timer_handle_);
}

void SteadyTimer::Impl::setPeriod(const WallDuration& period, bool reset)
{
  period_ = period;
  TimerManager<SteadyTime, WallDuration, SteadyTimerEvent>::global().setPeriod(timer_handle_, period, reset);
}


SteadyTimer::SteadyTimer(const SteadyTimerOptions& ops)
: impl_(new Impl)
{
  impl_->period_ = ops.period;
  impl_->callback_ = ops.callback;
  impl_->callback_queue_ = ops.callback_queue;
  impl_->tracked_object_ = ops.tracked_object;
  impl_->has_tracked_object_ = (ops.tracked_object != NULL);
  impl_->oneshot_ = ops.oneshot;
}

SteadyTimer::SteadyTimer(const SteadyTimer& rhs)
{
  impl_ = rhs.impl_;
}

SteadyTimer::~SteadyTimer()
{
}

void SteadyTimer::start()
{
  if (impl_)
  {
    impl_->start();
  }
}

void SteadyTimer::stop()
{
  if (impl_)
  {
    impl_->stop();
  }
}

bool SteadyTimer::hasPending()
{
  if (impl_)
  {
    return impl_->hasPending();
  }

  return false;
}

void SteadyTimer::setPeriod(const WallDuration& period, bool reset)
{
  if (impl_)
  {
    impl_->setPeriod(period, reset);
  }
}

}
