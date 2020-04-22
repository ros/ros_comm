/*
 * Copyright (C) 2020, Willow Garage, Inc.
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

// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
// (C) Copyright 2007-10 Anthony Williams
// (C) Copyright 2011-2012 Vicente J. Botet Escriba

#ifndef ROSCPP_INTERNAL_CONDITION_VARIABLE_H
#define ROSCPP_INTERNAL_CONDITION_VARIABLE_H

#include <boost/thread/condition_variable.hpp>

namespace ros {
namespace internal {

#if !defined(BOOST_THREAD_PLATFORM_PTHREAD) || \
    defined(BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC) || \
    defined(BOOST_THREAD_INTERNAL_CLOCK_IS_MONO)
using condition_variable_monotonic = boost::condition_variable;

#else

class condition_variable_monotonic {
private:
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
  pthread_mutex_t internal_mutex;
#endif
  pthread_cond_t cond;

public:
  condition_variable_monotonic() {
    int res;
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
    res = pthread_mutex_init(&internal_mutex, NULL);
    if (res)
    {
      boost::throw_exception(boost::thread_resource_error(res, "ros::internal::condition_variable_monotonic::condition_variable_monotonic() constructor failed in pthread_mutex_init"));
    }
#endif

    // res = boost::detail::monotonic_pthread_cond_init(cond);
    pthread_condattr_t attr;
    res = pthread_condattr_init(&attr);
    if (res == 0) {
      pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
      res = pthread_cond_init(&cond, &attr);
      pthread_condattr_destroy(&attr);
    }

    if (res)
    {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
      BOOST_VERIFY(!pthread_mutex_destroy(&internal_mutex));
#endif
      boost::throw_exception(boost::thread_resource_error(res, "ros::internal::condition_variable_monotonic::condition_variable() constructor failed in detail::monotonic_pthread_cond_init"));
    }
  }

  void notify_one() BOOST_NOEXCEPT
  {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
    boost::pthread::pthread_mutex_scoped_lock internal_lock(&internal_mutex);
#endif
    BOOST_VERIFY(!pthread_cond_signal(&cond));
  }

  void notify_all() BOOST_NOEXCEPT
  {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
    boost::pthread::pthread_mutex_scoped_lock internal_lock(&internal_mutex);
#endif
    BOOST_VERIFY(!pthread_cond_broadcast(&cond));
  }

  template <class Duration>
  boost::cv_status wait_until(
      boost::unique_lock<boost::mutex> &lock,
      const boost::chrono::time_point<boost::chrono::steady_clock, Duration> &t)
  {
    using namespace boost::chrono;
    typedef time_point<steady_clock, nanoseconds> nano_sys_tmpt;
    wait_until(lock,
               nano_sys_tmpt(ceil<nanoseconds>(t.time_since_epoch())));
    return steady_clock::now() < t ? boost::cv_status::no_timeout : boost::cv_status::timeout;
  }

  template <class Clock, class Duration>
  boost::cv_status wait_until(
      boost::unique_lock<boost::mutex> &lock,
      const boost::chrono::time_point<Clock, Duration> &t)
  {
    using namespace boost::chrono;
    steady_clock::time_point s_now = steady_clock::now();
    typename Clock::time_point c_now = Clock::now();
    wait_until(lock, s_now + ceil<nanoseconds>(t - c_now));
    return Clock::now() < t ? boost::cv_status::no_timeout : boost::cv_status::timeout;
  }

  template <class Rep, class Period>
  boost::cv_status wait_for(
      boost::unique_lock<boost::mutex> &lock,
      const boost::chrono::duration<Rep, Period> &d)
  {
    using namespace boost::chrono;
    steady_clock::time_point c_now = steady_clock::now();
    wait_until(lock, c_now + ceil<nanoseconds>(d));
    return steady_clock::now() - c_now < d ? boost::cv_status::no_timeout : boost::cv_status::timeout;
  }

  boost::cv_status wait_until(
      boost::unique_lock<boost::mutex> &lk,
      boost::chrono::time_point<boost::chrono::steady_clock, boost::chrono::nanoseconds> tp)
  {
    using namespace boost::chrono;
    nanoseconds d = tp.time_since_epoch();
    timespec ts = boost::detail::to_timespec(d);
    if (do_wait_until(lk, ts))
      return boost::cv_status::no_timeout;
    else
      return boost::cv_status::timeout;
  }

  void wait(boost::unique_lock<boost::mutex> &m)
  {
    int res = 0;
    {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
      boost::thread_cv_detail::lock_on_exit<boost::unique_lock<boost::mutex>> guard;
      boost::detail::interruption_checker check_for_interruption(&internal_mutex, &cond);
      pthread_mutex_t *the_mutex = &internal_mutex;
      guard.activate(m);
      res = pthread_cond_wait(&cond, the_mutex);
#if BOOST_VERSION >= 106500
      check_for_interruption.check();
      guard.deactivate();
#endif
#else
      pthread_mutex_t *the_mutex = m.mutex()->native_handle();
      res = pthread_cond_wait(&cond, the_mutex);
#endif
    }
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
    boost::this_thread::interruption_point();
#endif
    if (res && res != EINTR)
    {
      boost::throw_exception(boost::condition_error(res, "ros::internal::condition_variable_monotonic::wait failed in pthread_cond_wait"));
    }
  }

  bool do_wait_until(
      boost::unique_lock<boost::mutex> &m,
      struct timespec const &timeout)
  {
    int cond_res;
    {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
      boost::thread_cv_detail::lock_on_exit<boost::unique_lock<boost::mutex>> guard;
      boost::detail::interruption_checker check_for_interruption(&internal_mutex, &cond);
      pthread_mutex_t *the_mutex = &internal_mutex;
      guard.activate(m);
      cond_res = pthread_cond_timedwait(&cond, the_mutex, &timeout);
#if BOOST_VERSION >= 106500
      check_for_interruption.check();
      guard.deactivate();
#endif
#else
      pthread_mutex_t *the_mutex = m.mutex()->native_handle();
      cond_res = pthread_cond_timedwait(&cond, the_mutex, &timeout);
#endif
    }
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
    boost::this_thread::interruption_point();
#endif
    if (cond_res == ETIMEDOUT)
    {
      return false;
    }
    if (cond_res)
    {
      boost::throw_exception(boost::condition_error(cond_res, "ros::internal::condition_variable_monotonic::do_wait_until failed in pthread_cond_timedwait"));
    }
    return true;
  }
};
static_assert(
    sizeof(condition_variable_monotonic) == sizeof(boost::condition_variable),
    "sizeof(ros::internal::condition_variable_monotonic) != sizeof(boost::condition_variable)");

#endif

}  // namespace internal
}  // namespaec ros

#endif  // ROSCPP_INTERNAL_CONDITION_VARIABLE_H