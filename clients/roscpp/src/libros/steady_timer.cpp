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

// make sure we use CLOCK_MONOTONIC for the condition variable
#define BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC

#include "ros/steady_timer.h"
#include "ros/timer_manager.h"

namespace ros
{

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
