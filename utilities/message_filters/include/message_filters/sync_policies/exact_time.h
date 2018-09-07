/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef MESSAGE_FILTERS_SYNC_EXACT_TIME_H
#define MESSAGE_FILTERS_SYNC_EXACT_TIME_H

#include "message_filters/synchronizer.h"
#include "message_filters/connection.h"
#include "message_filters/null_types.h"
#include "message_filters/signal9.h"

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include <boost/bind.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/noncopyable.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/vector.hpp>

#include <ros/assert.h>
#include <ros/message_traits.h>
#include <ros/message_event.h>

#include <deque>
#include <vector>
#include <string>

namespace message_filters
{
namespace sync_policies
{

namespace mpl = boost::mpl;


template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
struct ExactTime : public PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
  typedef Synchronizer<ExactTime> Sync;
  typedef PolicyBase<M0, M1, M2, M3, M4, M5, M6, M7, M8> Super;
  typedef typename Super::Messages Messages;
  typedef typename Super::Signal Signal;
  typedef typename Super::Events Events;
  typedef typename Super::RealTypeCount RealTypeCount;
  typedef typename Super::M0Event M0Event;
  typedef typename Super::M1Event M1Event;
  typedef typename Super::M2Event M2Event;
  typedef typename Super::M3Event M3Event;
  typedef typename Super::M4Event M4Event;
  typedef typename Super::M5Event M5Event;
  typedef typename Super::M6Event M6Event;
  typedef typename Super::M7Event M7Event;
  typedef typename Super::M8Event M8Event;
  typedef boost::tuple<M0Event, M1Event, M2Event, M3Event, M4Event, M5Event, M6Event, M7Event, M8Event> Tuple;

  ExactTime(uint32_t queue_size)
  : parent_(0)
  , queue_size_(queue_size)
  {
  }

  ExactTime(const ExactTime& e)
  {
    *this = e;
  }

  ExactTime& operator=(const ExactTime& rhs)
  {
    parent_ = rhs.parent_;
    queue_size_ = rhs.queue_size_;
    last_signal_time_ = rhs.last_signal_time_;
    tuples_ = rhs.tuples_;

    return *this;
  }

  void initParent(Sync* parent)
  {
    parent_ = parent;
  }

  template<int i>
  void add(const typename mpl::at_c<Events, i>::type& evt)
  {
    ROS_ASSERT(parent_);

    namespace mt = ros::message_traits;

    boost::mutex::scoped_lock lock(mutex_);

    Tuple& t = tuples_[mt::TimeStamp<typename mpl::at_c<Messages, i>::type>::value(*evt.getMessage())];
    boost::get<i>(t) = evt;

    checkTuple(t);
  }

  template<class C>
  Connection registerDropCallback(const C& callback)
  {
  #ifndef _WIN32
    return drop_signal_.template addCallback(callback);
  #else
    return drop_signal_.addCallback(callback);
  #endif
  }

  template<class C>
  Connection registerDropCallback(C& callback)
  {
  #ifndef _WIN32
    return drop_signal_.template addCallback(callback);
  #else
    return drop_signal_.addCallback(callback);
  #endif
  }

  template<class C, typename T>
  Connection registerDropCallback(const C& callback, T* t)
  {
  #ifndef _WIN32
    return drop_signal_.template addCallback(callback, t);
  #else
    return drop_signal_.addCallback(callback, t);
  #endif
  }

  template<class C, typename T>
  Connection registerDropCallback(C& callback, T* t)
  {
  #ifndef _WIN32
    return drop_signal_.template addCallback(callback, t);
  #else
    return drop_signal_.addCallback(callback, t);
  #endif
  }

private:

  // assumes mutex_ is already locked
  void checkTuple(Tuple& t)
  {
    namespace mt = ros::message_traits;

    bool full = true;
    full = full && (bool)boost::get<0>(t).getMessage();
    full = full && (bool)boost::get<1>(t).getMessage();
    full = full && (RealTypeCount::value > 2 ? (bool)boost::get<2>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 3 ? (bool)boost::get<3>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 4 ? (bool)boost::get<4>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 5 ? (bool)boost::get<5>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 6 ? (bool)boost::get<6>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 7 ? (bool)boost::get<7>(t).getMessage() : true);
    full = full && (RealTypeCount::value > 8 ? (bool)boost::get<8>(t).getMessage() : true);

    if (full)
    {
      parent_->signal(boost::get<0>(t), boost::get<1>(t), boost::get<2>(t),
                       boost::get<3>(t), boost::get<4>(t), boost::get<5>(t),
                       boost::get<6>(t), boost::get<7>(t), boost::get<8>(t));

      last_signal_time_ = mt::TimeStamp<M0>::value(*boost::get<0>(t).getMessage());

      tuples_.erase(last_signal_time_);

      clearOldTuples();
    }

    if (queue_size_ > 0)
    {
      while (tuples_.size() > queue_size_)
      {
        Tuple& t2 = tuples_.begin()->second;
        drop_signal_.call(boost::get<0>(t2), boost::get<1>(t2), boost::get<2>(t2),
                          boost::get<3>(t2), boost::get<4>(t2), boost::get<5>(t2),
                          boost::get<6>(t2), boost::get<7>(t2), boost::get<8>(t2));
        tuples_.erase(tuples_.begin());
      }
    }
  }

  // assumes mutex_ is already locked
  void clearOldTuples()
  {
    typename M_TimeToTuple::iterator it = tuples_.begin();
    typename M_TimeToTuple::iterator end = tuples_.end();
    for (; it != end;)
    {
      if (it->first <= last_signal_time_)
      {
        typename M_TimeToTuple::iterator old = it;
        ++it;

        Tuple& t = old->second;
        drop_signal_.call(boost::get<0>(t), boost::get<1>(t), boost::get<2>(t),
                          boost::get<3>(t), boost::get<4>(t), boost::get<5>(t),
                          boost::get<6>(t), boost::get<7>(t), boost::get<8>(t));
        tuples_.erase(old);
      }
      else
      {
        // the map is sorted by time, so we can ignore anything after this if this one's time is ok
        break;
      }
    }
  }

private:
  Sync* parent_;

  uint32_t queue_size_;
  typedef std::map<ros::Time, Tuple> M_TimeToTuple;
  M_TimeToTuple tuples_;
  ros::Time last_signal_time_;

  Signal drop_signal_;

  boost::mutex mutex_;
};

} // namespace sync
} // namespace message_filters

#endif // MESSAGE_FILTERS_SYNC_EXACT_TIME_H

