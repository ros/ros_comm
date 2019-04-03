/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/* Author: Josh Faust */

/*
 * Test callback queue
 */

#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/message_deserializer.h>
#include <ros/subscription_queue.h>
#include <ros/subscription_callback_helper.h>
#include <ros/timer.h>

#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include "fake_message.h"

using namespace ros;

class CountingCallback : public CallbackInterface
{
public:
  CountingCallback()
  : count(0)
  {}

  virtual CallResult call()
  {
    boost::mutex::scoped_lock lock(mutex);
    ++count;

    return Success;
  }

  boost::mutex mutex;
  size_t count;
};
typedef boost::shared_ptr<CountingCallback> CountingCallbackPtr;

TEST(CallbackQueue, singleCallback)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  CallbackQueue queue;
  queue.addCallback(cb, 1);
  queue.callOne();

  EXPECT_EQ(cb->count, 1U);

  queue.addCallback(cb, 1);
  queue.callAvailable();

  EXPECT_EQ(cb->count, 2U);

  queue.callOne();
  EXPECT_EQ(cb->count, 2U);

  queue.callAvailable();
  EXPECT_EQ(cb->count, 2U);
}

TEST(CallbackQueue, multipleCallbacksCallAvailable)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  CallbackQueue queue;
  for (uint32_t i = 0; i < 1000; ++i)
  {
    queue.addCallback(cb, 1);
  }

  queue.callAvailable();

  EXPECT_EQ(cb->count, 1000U);
}

TEST(CallbackQueue, multipleCallbacksCallOne)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  CallbackQueue queue;
  for (uint32_t i = 0; i < 1000; ++i)
  {
    queue.addCallback(cb, 1);
  }

  for (uint32_t i = 0; i < 1000; ++i)
  {
    queue.callOne();
    EXPECT_EQ(cb->count, i + 1);
  }
}

TEST(CallbackQueue, remove)
{
  CountingCallbackPtr cb1(boost::make_shared<CountingCallback>());
  CountingCallbackPtr cb2(boost::make_shared<CountingCallback>());
  CallbackQueue queue;
  queue.addCallback(cb1, 1);
  queue.addCallback(cb2, 2);
  queue.removeByID(1);
  queue.callAvailable();

  EXPECT_EQ(cb1->count, 0U);
  EXPECT_EQ(cb2->count, 1U);
}

class SelfRemovingCallback : public CallbackInterface
{
public:
  SelfRemovingCallback(CallbackQueue* queue, uint64_t id)
  : count(0)
  , queue(queue)
  , id(id)
  {}

  virtual CallResult call()
  {
    ++count;

    queue->removeByID(id);

    return Success;
  }

  size_t count;

  CallbackQueue* queue;
  uint64_t id;
};
typedef boost::shared_ptr<SelfRemovingCallback> SelfRemovingCallbackPtr;

TEST(CallbackQueue, removeSelf)
{
  CallbackQueue queue;
  SelfRemovingCallbackPtr cb1(boost::make_shared<SelfRemovingCallback>(&queue, 1));
  CountingCallbackPtr cb2(boost::make_shared<CountingCallback>());
  queue.addCallback(cb1, 1);
  queue.addCallback(cb2, 1);
  queue.addCallback(cb2, 1);

  queue.callOne();

  queue.addCallback(cb2, 1);
  
  queue.callAvailable();

  EXPECT_EQ(cb1->count, 1U);
  EXPECT_EQ(cb2->count, 1U);
}

class RecursiveCallback : public CallbackInterface
{
public:
  RecursiveCallback(CallbackQueue* queue, bool use_available)
  : count(0)
  , queue(queue)
  , use_available(use_available)
  {}

  virtual CallResult call()
  {
    ++count;

    if (count < 3)
    {
      if (use_available)
      {
        queue->callAvailable();
      }
      else
      {
        queue->callOne();
      }
    }

    return Success;
  }

  size_t count;

  CallbackQueue* queue;
  bool use_available;
};
typedef boost::shared_ptr<RecursiveCallback> RecursiveCallbackPtr;

TEST(CallbackQueue, recursive1)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(boost::make_shared<RecursiveCallback>(&queue, true));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callAvailable();

  EXPECT_EQ(cb->count, 3U);
}

TEST(CallbackQueue, recursive2)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(boost::make_shared<RecursiveCallback>(&queue, false));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callOne();

  EXPECT_EQ(cb->count, 3U);
}

TEST(CallbackQueue, recursive3)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(boost::make_shared<RecursiveCallback>(&queue, false));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callAvailable();

  EXPECT_EQ(cb->count, 3U);
}

TEST(CallbackQueue, recursive4)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(boost::make_shared<RecursiveCallback>(&queue, true));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callOne();

  EXPECT_EQ(cb->count, 3U);
}

void callAvailableThread(CallbackQueue* queue, bool& done, boost::atomic<size_t>* num_calls,
                         ros::WallDuration call_timeout = ros::WallDuration(0.1))
{
  size_t i = 0;
  while (!done)
  {
    queue->callAvailable(call_timeout);
    ++i;
  }

  if (num_calls)
    num_calls->fetch_add(i);
}

size_t runThreadedTest(const CallbackInterfacePtr& cb,
    const boost::function<void(CallbackQueue*, bool&, boost::atomic<size_t>*, ros::WallDuration)>& threadFunc,
    size_t* num_calls = NULL, size_t num_threads = 10,
    ros::WallDuration duration = ros::WallDuration(5),
    ros::WallDuration pause_between_callbacks = ros::WallDuration(0),
    ros::WallDuration call_one_timeout = ros::WallDuration(0.1))
{
  CallbackQueue queue;
  boost::thread_group tg;
  bool done = false;
  boost::atomic<size_t> calls(0);

  for (uint32_t i = 0; i < num_threads; ++i)
  {
    tg.create_thread(boost::bind(threadFunc, &queue, boost::ref(done), &calls, call_one_timeout));
  }

  ros::WallTime start = ros::WallTime::now();
  size_t i = 0;
  while (ros::WallTime::now() - start < duration)
  {
    queue.addCallback(cb);
    ++i;
    if (!pause_between_callbacks.isZero())
      pause_between_callbacks.sleep();
  }

  while (!queue.isEmpty())
  {
    ros::WallDuration(0.01).sleep();
  }

  done = true;
  tg.join_all();

  if (num_calls)
    *num_calls = calls;

  return i;
}

TEST(CallbackQueue, threadedCallAvailable)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  size_t i = runThreadedTest(cb, callAvailableThread);
  ROS_INFO_STREAM(i);
  EXPECT_EQ(cb->count, i);
}

void callOneThread(CallbackQueue* queue, bool& done, boost::atomic<size_t>* num_calls,
    ros::WallDuration timeout = ros::WallDuration(0.1))
{
  size_t i = 0;
  while (!done)
  {
    queue->callOne(timeout);
    ++i;
  }

  if (num_calls)
    num_calls->fetch_add(i);
}

TEST(CallbackQueue, threadedCallOne)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  size_t i = runThreadedTest(cb, callOneThread);
  ROS_INFO_STREAM(i);
  EXPECT_EQ(cb->count, i);
}

class CountingSubscriptionQueue : public SubscriptionQueue
{
public:
    CountingSubscriptionQueue(const std::string& topic, int32_t queue_size,
                              bool allow_concurrent_callbacks)
        : SubscriptionQueue(topic, queue_size, allow_concurrent_callbacks),
          ready_count(0)
    {}

    virtual bool ready() {
      ready_count.fetch_add(1);
      return SubscriptionQueue::ready();
    }

    boost::atomic<size_t> ready_count;
};
typedef boost::shared_ptr<CountingSubscriptionQueue> CountingSubscriptionQueuePtr;

struct ThreadedCallOneSlowParams {
    ros::WallDuration callback_duration; // long-lasting callback
    size_t num_threads;
    ros::WallDuration call_one_timeout;
    ros::WallDuration test_duration;
    ros::WallDuration pause_between_callbacks;
};

class CallbackQueueParamTest : public ::testing::TestWithParam<ThreadedCallOneSlowParams>
{};

TEST_P(CallbackQueueParamTest, threadedCallOneSlow)
{
  // test for https://github.com/ros/ros_comm/issues/1545
  // "roscpp multithreaded spinners eat up CPU when callbacks take too long"

  const ThreadedCallOneSlowParams param = GetParam();
  const WallDuration& callback_duration = param.callback_duration; // long-lasting callback
  const size_t num_threads = param.num_threads;
  const ros::WallDuration call_one_timeout = param.call_one_timeout;
  const ros::WallDuration test_duration = param.test_duration;
  const ros::WallDuration pause_between_callbacks = param.pause_between_callbacks;
  // queue_size is chosen such that it is larger than the max number of callbacks we
  // are really able to process in 5 secs (since allow_concurrent_callbacks is false,
  // it is equal to the number of seconds the queue is running)
  const size_t queue_size = static_cast<size_t>(test_duration.toSec()) + 1;

  // create a subscription queue counting the number of ready() calls
  const CountingSubscriptionQueuePtr cb(
      boost::make_shared<CountingSubscriptionQueue>("test", queue_size, false));

  // create a slow subscription callback (takes 1 second to complete)
  const FakeSubHelperPtr helper(boost::make_shared<FakeSubHelper>());
  helper->cb_ = boost::bind(&ros::WallDuration::sleep, callback_duration);
  const MessageDeserializerPtr des(boost::make_shared<MessageDeserializer>(
      helper, SerializedMessage(), boost::shared_ptr<M_string>()));

  // fill the subscription queue to get max performance
  for (size_t i = 0; i < queue_size; ++i)
    cb->push(helper, des, false, VoidConstWPtr(), true);

  // keep filling the callback queue at maximum speed for 5 seconds and
  // spin up 10 processing threads until the queue is empty
  size_t num_call_one_calls = 0;
  const size_t num_callbacks_to_call = runThreadedTest(cb, callOneThread, &num_call_one_calls,
      num_threads, test_duration, pause_between_callbacks, call_one_timeout);

  const uint32_t num_callbacks_called = helper->calls_;
  const size_t num_ready_called = cb->ready_count;

  // what should happen: even though we have multiple processing threads,
  // the subscription queue has a per-topic lock which prevents multiple threads from
  // processing the topic's callbacks simultaneously; so even though there were
  // tens of thousands of callbacks in the callback queue, we only got time to process
  // less than 5 of them because queue_size is quite limited (3), so most callbacks
  // get thrown away during processing of the slow callbacks

  // we test the number of SubscriptionQueue::ready() calls to see how often do the
  // idle threads ask for more work; with bug 1545 unfixed, this gets to millions of
  // calls which acts as a busy-wait; with the bug fixed, the number should not be
  // higher than number of callbacks (~ 80k), since each newly added callback should
  // wake one idle thread and let it ask for work

  ROS_INFO_STREAM("Callback queue processed " <<
                  num_callbacks_called << " callbacks out of " << num_callbacks_to_call);
  ROS_INFO_STREAM("callOne() was called " << num_call_one_calls << " times.");
  ROS_INFO_STREAM("ready() was called " << num_ready_called << " times.");

  EXPECT_EQ(num_callbacks_called, queue_size);
  EXPECT_LE(num_call_one_calls, 2 * num_callbacks_to_call + num_threads * (1/call_one_timeout.toSec()) * queue_size);
}

INSTANTIATE_TEST_CASE_P(slow, CallbackQueueParamTest, ::testing::Values(
  //ThreadedCallOneSlowParams{callback_duration,        num_threads, call_one_timeout,       test_duration,        pause_between_callbacks}
    ThreadedCallOneSlowParams{ros::WallDuration(1.0),   10,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0)},
    ThreadedCallOneSlowParams{ros::WallDuration(1.0),   10,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0.1)},
    ThreadedCallOneSlowParams{ros::WallDuration(1.0),   10,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0.001)},
    ThreadedCallOneSlowParams{ros::WallDuration(0.1),   10,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0)},
    ThreadedCallOneSlowParams{ros::WallDuration(0.001), 10,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0)},
    ThreadedCallOneSlowParams{ros::WallDuration(1.0),    1,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0)},
    ThreadedCallOneSlowParams{ros::WallDuration(1.0),    2,          ros::WallDuration(0.1), ros::WallDuration(2), ros::WallDuration(0)}
    ));

// this class is just an ugly hack
// to access the constructor Timer(TimerOptions)
namespace ros
{
class NodeHandle
{
public:
  static Timer createTimer(const TimerOptions& ops)
  {
    return Timer(ops);
  }
};
}

void dummyTimer(const ros::TimerEvent&)
{
}

CallbackQueueInterface* recursiveTimerQueue;

void recursiveTimer(const ros::TimerEvent&)
{
  // wait until the timer is TimerRecreationCallback is garbaged
  WallDuration(2).sleep();

  TimerOptions ops(Duration(0.1), dummyTimer, recursiveTimerQueue, false, false);
  Timer t = ros::NodeHandle::createTimer(ops);
  t.start();
}

class TimerRecursionCallback : public CallbackInterface
{
public:
  TimerRecursionCallback(CallbackQueueInterface* _queue)
  : queue(_queue)
  {}

  virtual CallResult call()
  {
    TimerOptions ops(Duration(0.1), recursiveTimer, queue, false, false);
    Timer t = ros::NodeHandle::createTimer(ops);
    t.start();

    // wait until the recursiveTimer has been fired
    WallDuration(1).sleep();

    return Success;
  }

  CallbackQueueInterface* queue;
};
typedef boost::shared_ptr<TimerRecursionCallback> TimerRecursionCallbackPtr;

TEST(CallbackQueue, recursiveTimer)
{
  // ensure that the test does not dead-lock, see #3867
  ros::Time::init();
  CallbackQueue queue;
  recursiveTimerQueue = &queue;
  TimerRecursionCallbackPtr cb(boost::make_shared<TimerRecursionCallback>(&queue));
  queue.addCallback(cb, 1);

  boost::thread_group tg;
  bool done = false;
  boost::atomic<size_t> calls(0);

  for (uint32_t i = 0; i < 2; ++i)
  {
    tg.create_thread(boost::bind(callOneThread, &queue, boost::ref(done), &calls, ros::WallDuration(0.1)));
  }

  while (!queue.isEmpty())
  {
    ros::WallDuration(0.01).sleep();
  }

  done = true;
  tg.join_all();
}

class ConditionObject
{
public:
  ConditionObject(CallbackQueue * _queue)
  : id(0), queue(_queue) {
    condition_sync.store(true);
    condition_one.store(false);
    condition_stop.store(false);
  }

  void add();

  unsigned long id;
  CallbackQueue * queue;
  boost::atomic<bool> condition_one;
  boost::atomic<bool> condition_sync;
  boost::atomic<bool> condition_stop;
};

class RaceConditionCallback : public CallbackInterface
{
public:
  RaceConditionCallback(ConditionObject * _condition_object, unsigned long * _id)
  : condition_object(_condition_object), id(_id)
  {}

  virtual CallResult call()
  {
    condition_object->condition_one.store(false);
    return Success;
  }

  ConditionObject * condition_object;
  unsigned long * id;
};

void ConditionObject::add()
{
  while(!condition_stop.load())
  {
    if(condition_sync.load() && queue->isEmpty())
    {
      condition_one.store(true);
      id++;
      queue->addCallback(boost::make_shared<RaceConditionCallback>(this, &id), id);
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(1));
  }
}

TEST(CallbackQueue, raceConditionCallback)
{
  CallbackQueue queue;
  ConditionObject condition_object(&queue);

  boost::thread t(boost::bind(&ConditionObject::add, &condition_object));
  for(unsigned int i = 0; i < 1000000; ++i)
  {
    condition_object.condition_sync.store(false);
    if (queue.callOne() == CallbackQueue::Called)
    {
      if(condition_object.condition_one.load())
      {
        condition_object.condition_stop.store(true);
        ASSERT_FALSE(condition_object.condition_one.load());
      }
    }

    queue.clear();
    condition_object.condition_sync.store(true);
  }
  condition_object.condition_stop.store(true);
  t.join();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



