#ifndef BOOST_161_THREAD_CONDITION_VARIABLE_PTHREAD_HPP
#define BOOST_161_THREAD_CONDITION_VARIABLE_PTHREAD_HPP
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
// (C) Copyright 2007-10 Anthony Williams
// (C) Copyright 2011-2012 Vicente J. Botet Escriba

// make sure we include our backported version first!!
#include "boost_161_pthread_condition_variable_fwd.h"

// include upstream
#include <boost/thread/pthread/condition_variable.hpp>

#include <boost/config/abi_prefix.hpp>

namespace boost_161
{

    inline void condition_variable::wait(boost::unique_lock<mutex>& m)
    {
#if defined BOOST_THREAD_THROW_IF_PRECONDITION_NOT_SATISFIED
        if(! m.owns_lock())
        {
            boost::throw_exception(boost::condition_error(-1, "boost::condition_variable::wait() failed precondition mutex not owned"));
        }
#endif
        int res=0;
        {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
            boost::thread_cv_detail::lock_on_exit<boost::unique_lock<mutex> > guard;
            detail::interruption_checker check_for_interruption(&internal_mutex,&cond);
            pthread_mutex_t* the_mutex = &internal_mutex;
            guard.activate(m);
#else
            pthread_mutex_t* the_mutex = m.mutex()->native_handle();
#endif
            res = pthread_cond_wait(&cond,the_mutex);
        }
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
        boost::this_thread::interruption_point();
#endif
        if(res && res != EINTR)
        {
            boost::throw_exception(boost::condition_error(res, "boost::condition_variable::wait failed in pthread_cond_wait"));
        }
    }

    inline bool condition_variable::do_wait_until(
                boost::unique_lock<mutex>& m,
                struct timespec const &timeout)
    {
#if defined BOOST_THREAD_THROW_IF_PRECONDITION_NOT_SATISFIED
        if (!m.owns_lock())
        {
            boost::throw_exception(boost::condition_error(EPERM, "boost::condition_variable::do_wait_until() failed precondition mutex not owned"));
        }
#endif
        int cond_res;
        {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
            boost::thread_cv_detail::lock_on_exit<boost::unique_lock<mutex> > guard;
            detail::interruption_checker check_for_interruption(&internal_mutex,&cond);
            pthread_mutex_t* the_mutex = &internal_mutex;
            guard.activate(m);
#else
            pthread_mutex_t* the_mutex = m.mutex()->native_handle();
#endif
            cond_res=pthread_cond_timedwait(&cond,the_mutex,&timeout);
        }
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
        boost::this_thread::interruption_point();
#endif
        if(cond_res==ETIMEDOUT)
        {
            return false;
        }
        if(cond_res)
        {
            boost::throw_exception(boost::condition_error(cond_res, "boost::condition_variable::do_wait_until failed in pthread_cond_timedwait"));
        }
        return true;
    }

    inline void condition_variable::notify_one() BOOST_NOEXCEPT
    {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
        boost::pthread::pthread_mutex_scoped_lock internal_lock(&internal_mutex);
#endif
        BOOST_VERIFY(!pthread_cond_signal(&cond));
    }

    inline void condition_variable::notify_all() BOOST_NOEXCEPT
    {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
        boost::pthread::pthread_mutex_scoped_lock internal_lock(&internal_mutex);
#endif
        BOOST_VERIFY(!pthread_cond_broadcast(&cond));
    }

    class condition_variable_any
    {
        pthread_mutex_t internal_mutex;
        pthread_cond_t cond;

    public:
        BOOST_THREAD_NO_COPYABLE(condition_variable_any)
        condition_variable_any()
        {
            int const res=pthread_mutex_init(&internal_mutex,NULL);
            if(res)
            {
                boost::throw_exception(boost::thread_resource_error(res, "boost::condition_variable_any::condition_variable_any() failed in pthread_mutex_init"));
            }
            int const res2 = detail_161::monotonic_pthread_cond_init(cond);
            if(res2)
            {
                BOOST_VERIFY(!pthread_mutex_destroy(&internal_mutex));
                boost::throw_exception(boost::thread_resource_error(res2, "boost::condition_variable_any::condition_variable_any() failed in detail::monotonic_pthread_cond_init"));
            }
        }
        ~condition_variable_any()
        {
            BOOST_VERIFY(!pthread_mutex_destroy(&internal_mutex));
            BOOST_VERIFY(!pthread_cond_destroy(&cond));
        }

        template<typename lock_type>
        void wait(lock_type& m)
        {
            int res=0;
            {
                boost::thread_cv_detail::lock_on_exit<lock_type> guard;
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
                detail::interruption_checker check_for_interruption(&internal_mutex,&cond);
#else
                boost::pthread::pthread_mutex_scoped_lock check_for_interruption(&internal_mutex);
#endif
                guard.activate(m);
                res=pthread_cond_wait(&cond,&internal_mutex);
            }
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
            boost::this_thread::interruption_point();
#endif
            if(res)
            {
                boost::throw_exception(boost::condition_error(res, "boost::condition_variable_any::wait() failed in pthread_cond_wait"));
            }
        }

        template<typename lock_type,typename predicate_type>
        void wait(lock_type& m,predicate_type pred)
        {
            while(!pred()) wait(m);
        }

#if defined BOOST_THREAD_USES_DATETIME
        template<typename lock_type>
        bool timed_wait(lock_type& m,boost::system_time const& abs_time)
        {
            struct timespec const timeout=boost::detail::to_timespec(abs_time);
            return do_wait_until(m, timeout);
        }
        template<typename lock_type>
        bool timed_wait(lock_type& m,boost::xtime const& abs_time)
        {
            return timed_wait(m,boost::system_time(abs_time));
        }

        template<typename lock_type,typename duration_type>
        bool timed_wait(lock_type& m,duration_type const& wait_duration)
        {
            return timed_wait(m,boost::get_system_time()+wait_duration);
        }

        template<typename lock_type,typename predicate_type>
        bool timed_wait(lock_type& m,boost::system_time const& abs_time, predicate_type pred)
        {
            while (!pred())
            {
                if(!timed_wait(m, abs_time))
                    return pred();
            }
            return true;
        }

        template<typename lock_type,typename predicate_type>
        bool timed_wait(lock_type& m,boost::xtime const& abs_time, predicate_type pred)
        {
            return timed_wait(m,boost::system_time(abs_time),pred);
        }

        template<typename lock_type,typename duration_type,typename predicate_type>
        bool timed_wait(lock_type& m,duration_type const& wait_duration,predicate_type pred)
        {
            return timed_wait(m,boost::get_system_time()+wait_duration,pred);
        }
#endif
#ifndef BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC

#ifdef BOOST_THREAD_USES_CHRONO
        template <class lock_type,class Duration>
        boost::cv_status
        wait_until(
                lock_type& lock,
                const boost::chrono::time_point<chrono::system_clock, Duration>& t)
        {
          typedef boost::chrono::time_point<boost::chrono::system_clock, boost::chrono::nanoseconds> nano_sys_tmpt;
          wait_until(lock,
                        nano_sys_tmpt(ceil<boost::chrono::nanoseconds>(t.time_since_epoch())));
          return boost::chrono::system_clock::now() < t ? boost::cv_status::no_timeout :
                                             boost::cv_status::timeout;
        }

        template <class lock_type, class Clock, class Duration>
        boost::cv_status
        wait_until(
                lock_type& lock,
                const boost::chrono::time_point<Clock, Duration>& t)
        {
          boost::chrono::system_clock::time_point     s_now = boost::chrono::system_clock::now();
          typename Clock::time_point  c_now = Clock::now();
          wait_until(lock, s_now + ceil<boost::chrono::nanoseconds>(t - c_now));
          return Clock::now() < t ? boost::cv_status::no_timeout : boost::cv_status::timeout;
        }

        template <class lock_type, class Rep, class Period>
        boost::cv_status
        wait_for(
                lock_type& lock,
                const boost::chrono::duration<Rep, Period>& d)
        {
          boost::chrono::system_clock::time_point s_now = boost::chrono::system_clock::now();
          boost::chrono::steady_clock::time_point c_now = boost::chrono::steady_clock::now();
          wait_until(lock, s_now + ceil<boost::chrono::nanoseconds>(d));
          return boost::chrono::steady_clock::now() - c_now < d ? boost::cv_status::no_timeout :
                                                   boost::cv_status::timeout;

        }

        template <class lock_type>
        boost::cv_status wait_until(
            lock_type& lk,
            boost::chrono::time_point<boost::chrono::system_clock, boost::chrono::nanoseconds> tp)
        {
            boost::chrono::nanoseconds d = tp.time_since_epoch();
            timespec ts = boost::detail::to_timespec(d);
            if (do_wait_until(lk, ts)) return boost::cv_status::no_timeout;
            else return boost::cv_status::timeout;
        }
#endif
#else // defined BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC
#ifdef BOOST_THREAD_USES_CHRONO

        template <class lock_type, class Duration>
        boost::cv_status
        wait_until(
            lock_type& lock,
            const boost::chrono::time_point<boost::chrono::steady_clock, Duration>& t)
        {
            typedef boost::chrono::time_point<boost::chrono::steady_clock, boost::chrono::nanoseconds> nano_sys_tmpt;
            wait_until(lock,
                        nano_sys_tmpt(ceil<boost::chrono::nanoseconds>(t.time_since_epoch())));
            return boost::chrono::steady_clock::now() < t ? boost::cv_status::no_timeout :
                                             boost::cv_status::timeout;
        }

        template <class lock_type, class Clock, class Duration>
        boost::cv_status
        wait_until(
            lock_type& lock,
            const boost::chrono::time_point<Clock, Duration>& t)
        {
            boost::chrono::steady_clock::time_point     s_now = boost::chrono::steady_clock::now();
            typename Clock::time_point  c_now = Clock::now();
            wait_until(lock, s_now + ceil<boost::chrono::nanoseconds>(t - c_now));
            return Clock::now() < t ? boost::cv_status::no_timeout : boost::cv_status::timeout;
        }

        template <class lock_type, class Rep, class Period>
        boost::cv_status
        wait_for(
            lock_type& lock,
            const boost::chrono::duration<Rep, Period>& d)
        {
            boost::chrono::steady_clock::time_point c_now = boost::chrono::steady_clock::now();
            wait_until(lock, c_now + ceil<boost::chrono::nanoseconds>(d));
            return boost::chrono::steady_clock::now() - c_now < d ? boost::cv_status::no_timeout :
                                                   boost::cv_status::timeout;
        }

        template <class lock_type>
        inline boost::cv_status wait_until(
            lock_type& lock,
            boost::chrono::time_point<boost::chrono::steady_clock, boost::chrono::nanoseconds> tp)
        {
            boost::chrono::nanoseconds d = tp.time_since_epoch();
            timespec ts = boost::detail::to_timespec(d);
            if (do_wait_until(lock, ts)) return boost::cv_status::no_timeout;
            else return boost::cv_status::timeout;
        }

#endif
#endif // defined BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC

#ifdef BOOST_THREAD_USES_CHRONO
        template <class lock_type, class Clock, class Duration, class Predicate>
        bool
        wait_until(
                lock_type& lock,
                const boost::chrono::time_point<Clock, Duration>& t,
                Predicate pred)
        {
            while (!pred())
            {
                if (wait_until(lock, t) == boost::cv_status::timeout)
                    return pred();
            }
            return true;
        }

        template <class lock_type, class Rep, class Period, class Predicate>
        bool
        wait_for(
                lock_type& lock,
                const boost::chrono::duration<Rep, Period>& d,
                Predicate pred)
        {
          return wait_until(lock, boost::chrono::steady_clock::now() + d, boost::move(pred));
        }
#endif

        void notify_one() BOOST_NOEXCEPT
        {
            boost::pthread::pthread_mutex_scoped_lock internal_lock(&internal_mutex);
            BOOST_VERIFY(!pthread_cond_signal(&cond));
        }

        void notify_all() BOOST_NOEXCEPT
        {
            boost::pthread::pthread_mutex_scoped_lock internal_lock(&internal_mutex);
            BOOST_VERIFY(!pthread_cond_broadcast(&cond));
        }
    private: // used by boost::thread::try_join_until

        template <class lock_type>
        bool do_wait_until(
          lock_type& m,
          struct timespec const &timeout)
        {
          int res=0;
          {
              boost::thread_cv_detail::lock_on_exit<lock_type> guard;
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
              detail::interruption_checker check_for_interruption(&internal_mutex,&cond);
#else
              boost::pthread::pthread_mutex_scoped_lock check_for_interruption(&internal_mutex);
#endif
              guard.activate(m);
              res=pthread_cond_timedwait(&cond,&internal_mutex,&timeout);
          }
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
          boost::this_thread::interruption_point();
#endif
          if(res==ETIMEDOUT)
          {
              return false;
          }
          if(res)
          {
              boost::throw_exception(boost::condition_error(res, "boost::condition_variable_any::do_wait_until() failed in pthread_cond_timedwait"));
          }
          return true;
        }
    };

}

#include <boost/config/abi_suffix.hpp>

#endif
