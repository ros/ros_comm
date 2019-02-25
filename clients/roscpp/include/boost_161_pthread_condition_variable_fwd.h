#ifndef BOOST_161_THREAD_PTHREAD_CONDITION_VARIABLE_FWD_HPP
#define BOOST_161_THREAD_PTHREAD_CONDITION_VARIABLE_FWD_HPP
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
// (C) Copyright 2007-8 Anthony Williams
// (C) Copyright 2011-2012 Vicente J. Botet Escriba

// include upstream
#include <boost/thread/pthread/condition_variable_fwd.hpp>

#include <boost/config/abi_prefix.hpp>

namespace boost_161
{
  namespace detail = boost::detail;

  namespace detail_161 {
    inline int monotonic_pthread_cond_init(pthread_cond_t& cond) {

#ifdef BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC
            pthread_condattr_t attr;
            int res = pthread_condattr_init(&attr);
            if (res)
            {
              return res;
            }
            pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
            res=pthread_cond_init(&cond,&attr);
            pthread_condattr_destroy(&attr);
            return res;
#else
            return pthread_cond_init(&cond,NULL);
#endif

    }
  }

    class condition_variable
    {
    private:
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
        pthread_mutex_t internal_mutex;
#endif
        pthread_cond_t cond;

    public:
    //private: // used by boost::thread::try_join_until

        inline bool do_wait_until(
            boost::unique_lock<mutex>& lock,
            struct timespec const &timeout);

        bool do_wait_for(
            boost::unique_lock<mutex>& lock,
            struct timespec const &timeout)
        {
#if ! defined BOOST_THREAD_USEFIXES_TIMESPEC
            return do_wait_until(lock, boost::detail::timespec_plus(timeout, boost::detail::timespec_now()));
#elif ! defined BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC
            //using namespace chrono;
            //nanoseconds ns = chrono::system_clock::now().time_since_epoch();

            struct timespec ts = boost::detail::timespec_now_realtime();
            //ts.tv_sec = static_cast<long>(chrono::duration_cast<chrono::seconds>(ns).count());
            //ts.tv_nsec = static_cast<long>((ns - chrono::duration_cast<chrono::seconds>(ns)).count());
            return do_wait_until(lock, boost::detail::timespec_plus(timeout, ts));
#else
            // old behavior was fine for monotonic
            return do_wait_until(lock, boost::detail::timespec_plus(timeout, boost::detail::timespec_now_realtime()));
#endif
        }

    public:
      BOOST_THREAD_NO_COPYABLE(condition_variable)
        condition_variable()
        {
            int res;
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
            res=pthread_mutex_init(&internal_mutex,NULL);
            if(res)
            {
                boost::throw_exception(boost::thread_resource_error(res, "boost::condition_variable::condition_variable() constructor failed in pthread_mutex_init"));
            }
#endif
            res = detail_161::monotonic_pthread_cond_init(cond);
            if (res)
            {
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
                BOOST_VERIFY(!pthread_mutex_destroy(&internal_mutex));
#endif
                boost::throw_exception(boost::thread_resource_error(res, "boost::condition_variable::condition_variable() constructor failed in detail::monotonic_pthread_cond_init"));
            }
        }
        ~condition_variable()
        {
            int ret;
#if defined BOOST_THREAD_PROVIDES_INTERRUPTIONS
            do {
              ret = pthread_mutex_destroy(&internal_mutex);
            } while (ret == EINTR);
            BOOST_ASSERT(!ret);
#endif
            do {
              ret = pthread_cond_destroy(&cond);
            } while (ret == EINTR);
            BOOST_ASSERT(!ret);
        }

        void wait(boost::unique_lock<mutex>& m);

        template<typename predicate_type>
        void wait(boost::unique_lock<mutex>& m,predicate_type pred)
        {
            while(!pred()) wait(m);
        }

#if defined BOOST_THREAD_USES_DATETIME
        inline bool timed_wait(
            boost::unique_lock<mutex>& m,
            boost::system_time const& abs_time)
        {
#if defined BOOST_THREAD_WAIT_BUG
            struct timespec const timeout=detail::to_timespec(abs_time + BOOST_THREAD_WAIT_BUG);
            return do_wait_until(m, timeout);
#else
            struct timespec const timeout=detail::to_timespec(abs_time);
            return do_wait_until(m, timeout);
#endif
        }
        bool timed_wait(
            boost::unique_lock<mutex>& m,
            boost::xtime const& abs_time)
        {
            return timed_wait(m,boost::system_time(abs_time));
        }

        template<typename duration_type>
        bool timed_wait(
            boost::unique_lock<mutex>& m,
            duration_type const& wait_duration)
        {
            if (wait_duration.is_pos_infinity())
            {
                wait(m); // or do_wait(m,detail::timeout::sentinel());
                return true;
            }
            if (wait_duration.is_special())
            {
                return true;
            }
            return timed_wait(m,boost::get_system_time()+wait_duration);
        }

        template<typename predicate_type>
        bool timed_wait(
            boost::unique_lock<mutex>& m,
            boost::system_time const& abs_time,predicate_type pred)
        {
            while (!pred())
            {
                if(!timed_wait(m, abs_time))
                    return pred();
            }
            return true;
        }

        template<typename predicate_type>
        bool timed_wait(
            boost::unique_lock<mutex>& m,
            boost::xtime const& abs_time,predicate_type pred)
        {
            return timed_wait(m,boost::system_time(abs_time),pred);
        }

        template<typename duration_type,typename predicate_type>
        bool timed_wait(
            boost::unique_lock<mutex>& m,
            duration_type const& wait_duration,predicate_type pred)
        {
            if (wait_duration.is_pos_infinity())
            {
                while (!pred())
                {
                  wait(m); // or do_wait(m,detail::timeout::sentinel());
                }
                return true;
            }
            if (wait_duration.is_special())
            {
                return pred();
            }
            return timed_wait(m,boost::get_system_time()+wait_duration,pred);
        }
#endif

#ifndef BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC

#ifdef BOOST_THREAD_USES_CHRONO

        template <class Duration>
        boost::cv_status
        wait_until(
                boost::unique_lock<mutex>& lock,
                const boost::chrono::time_point<boost::chrono::system_clock, Duration>& t)
        {
          typedef boost::chrono::time_point<boost::chrono::system_clock, boost::chrono::nanoseconds> nano_sys_tmpt;
          wait_until(lock,
                        nano_sys_tmpt(ceil<boost::chrono::nanoseconds>(t.time_since_epoch())));
          return boost::chrono::system_clock::now() < t ? boost::cv_status::no_timeout :
                                             boost::cv_status::timeout;
        }

        template <class Clock, class Duration>
        boost::cv_status
        wait_until(
                boost::unique_lock<mutex>& lock,
                const boost::chrono::time_point<Clock, Duration>& t)
        {
          boost::chrono::system_clock::time_point     s_now = boost::chrono::system_clock::now();
          typename Clock::time_point  c_now = Clock::now();
          wait_until(lock, s_now + ceil<boost::chrono::nanoseconds>(t - c_now));
          return Clock::now() < t ? boost::cv_status::no_timeout : boost::cv_status::timeout;
        }



        template <class Rep, class Period>
        boost::cv_status
        wait_for(
                boost::unique_lock<mutex>& lock,
                const boost::chrono::duration<Rep, Period>& d)
        {
          boost::chrono::system_clock::time_point s_now = boost::chrono::system_clock::now();
          boost::chrono::steady_clock::time_point c_now = boost::chrono::steady_clock::now();
          wait_until(lock, s_now + ceil<boost::chrono::nanoseconds>(d));
          return boost::chrono::steady_clock::now() - c_now < d ? boost::cv_status::no_timeout :
                                                   boost::cv_status::timeout;

        }

        inline boost::cv_status wait_until(
            boost::unique_lock<mutex>& lk,
            boost::chrono::time_point<boost::chrono::system_clock, boost::chrono::nanoseconds> tp)
        {
            boost::chrono::nanoseconds d = tp.time_since_epoch();
            timespec ts = detail::to_timespec(d);
            if (do_wait_until(lk, ts)) return boost::cv_status::no_timeout;
            else return boost::cv_status::timeout;
        }
#endif

#else // defined BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC
#ifdef BOOST_THREAD_USES_CHRONO

        template <class Duration>
        boost::cv_status
        wait_until(
              boost::unique_lock<mutex>& lock,
              const boost::chrono::time_point<boost::chrono::steady_clock, Duration>& t)
        {
            typedef time_point<boost::chrono::steady_clock, boost::chrono::nanoseconds> nano_sys_tmpt;
            wait_until(lock,
                        nano_sys_tmpt(ceil<boost::chrono::nanoseconds>(t.time_since_epoch())));
            return boost::chrono::steady_clock::now() < t ? boost::cv_status::no_timeout :
                                             boost::cv_status::timeout;
        }

        template <class Clock, class Duration>
        boost::cv_status
        wait_until(
            boost::unique_lock<mutex>& lock,
            const boost::chrono::time_point<Clock, Duration>& t)
        {
            boost::chrono::steady_clock::time_point     s_now = boost::chrono::steady_clock::now();
            typename Clock::time_point  c_now = Clock::now();
            wait_until(lock, s_now + ceil<boost::chrono::nanoseconds>(t - c_now));
            return Clock::now() < t ? boost::cv_status::no_timeout : boost::cv_status::timeout;
        }

        template <class Rep, class Period>
        boost::cv_status
        wait_for(
            boost::unique_lock<mutex>& lock,
            const boost::chrono::duration<Rep, Period>& d)
        {
            boost::chrono::steady_clock::time_point c_now = boost::chrono::steady_clock::now();
            wait_until(lock, c_now + ceil<boost::chrono::nanoseconds>(d));
            return boost::chrono::steady_clock::now() - c_now < d ? boost::cv_status::no_timeout :
                                                   boost::cv_status::timeout;
        }

        inline boost::cv_status wait_until(
            boost::unique_lock<mutex>& lk,
            boost::chrono::time_point<boost::chrono::steady_clock, boost::chrono::nanoseconds> tp)
        {
            nanoseconds d = tp.time_since_epoch();
            timespec ts = detail::to_timespec(d);
            if (do_wait_until(lk, ts)) return boost::cv_status::no_timeout;
            else return boost::cv_status::timeout;
        }
#endif

#endif // defined BOOST_THREAD_HAS_CONDATTR_SET_CLOCK_MONOTONIC

#ifdef BOOST_THREAD_USES_CHRONO
        template <class Clock, class Duration, class Predicate>
        bool
        wait_until(
                boost::unique_lock<mutex>& lock,
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

        template <class Rep, class Period, class Predicate>
        bool
        wait_for(
                boost::unique_lock<mutex>& lock,
                const boost::chrono::duration<Rep, Period>& d,
                Predicate pred)
        {
          return wait_until(lock, boost::chrono::steady_clock::now() + d, boost::move(pred));
        }
#endif

#define BOOST_THREAD_DEFINES_CONDITION_VARIABLE_NATIVE_HANDLE
        typedef pthread_cond_t* native_handle_type;
        native_handle_type native_handle()
        {
            return &cond;
        }

        void notify_one() BOOST_NOEXCEPT;
        void notify_all() BOOST_NOEXCEPT;


    };

    BOOST_THREAD_DECL void notify_all_at_thread_exit(condition_variable& cond, boost::unique_lock<mutex> lk);

}


#include <boost/config/abi_suffix.hpp>

#endif
