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
#include <ros/timer.h>

#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

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

void callAvailableThread(CallbackQueue* queue, bool& done)
{
  while (!done)
  {
    queue->callAvailable(ros::WallDuration(0.1));
  }
}


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

class BlockingCallback : public CallbackInterface
{
public:
  BlockingCallback()
  : count(0)
  {}

  virtual CallResult call()
  {
    mutex_.lock();
    ++count;

    return Success;
  }

  boost::mutex mutex_;
  size_t count;
};
typedef boost::shared_ptr<BlockingCallback> BlockingCallbackPtr;


// This test checks whether removing callbacks by an id doesn't block if one of those callback is being executed.
TEST(CallbackQueue, removeCallbackWhileExecuting)
{
  const uint64_t owner_id = 1;
  const uint64_t unrelated_id = 2;

  CallbackQueue queue;
  BlockingCallbackPtr cb1(boost::make_shared<BlockingCallback>());
  CountingCallbackPtr cb2(boost::make_shared<CountingCallback>());
  CountingCallbackPtr cb3(boost::make_shared<CountingCallback>());

  cb1->mutex_.lock();  // lock the mutex to ensure the blocking callback will stall processing of callback queue.

  queue.addCallback(cb1, owner_id); // Add the blocking callback.

  // Now, we need to serve the callback queue from another thread.
  bool done = false;
  boost::thread t = boost::thread(boost::bind(&callAvailableThread, &queue, boost::ref(done)));

  ros::WallDuration(1.0).sleep(); // Callback 1 should be being served now.

  queue.addCallback(cb2, unrelated_id); // Add a second callback with different owner.
  queue.addCallback(cb3, owner_id);     // Add a third with same owner, this one should never get executed.

  // Now try to remove the callback that's being executed.
  queue.removeByID(owner_id);  // This should not block because cb1 is being served, it should prevent cb3 from running.

  ros::WallDuration(1.0).sleep();

  // The removeByID should not block, so now we can unblock the blocking callback.
  cb1->mutex_.unlock(); // This allows processing of cb1 to continue.

  while (!queue.isEmpty())  // Wait until the queue is empty.
  {
    ros::WallDuration(0.01).sleep();
  }

  // Properly shut down our callback serving thread.
  done = true;
  t.join();

  EXPECT_EQ(cb1->count, 1U);
  EXPECT_EQ(cb2->count, 1U);
  EXPECT_EQ(cb3->count, 0U);

  cb1->mutex_.unlock();  // Ensure the mutex is unlocked before destruction.
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

size_t runThreadedTest(const CountingCallbackPtr& cb, const boost::function<void(CallbackQueue*, bool&)>& threadFunc)
{
  CallbackQueue queue;
  boost::thread_group tg;
  bool done = false;

  for (uint32_t i = 0; i < 10; ++i)
  {
    tg.create_thread(boost::bind(threadFunc, &queue, boost::ref(done)));
  }

  ros::WallTime start = ros::WallTime::now();
  size_t i = 0;
  while (ros::WallTime::now() - start < ros::WallDuration(5))
  {
    queue.addCallback(cb);
    ++i;
  }

  while (!queue.isEmpty())
  {
    ros::WallDuration(0.01).sleep();
  }

  done = true;
  tg.join_all();

  return i;
}

TEST(CallbackQueue, threadedCallAvailable)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  size_t i = runThreadedTest(cb, callAvailableThread);
  ROS_INFO_STREAM(i);
  EXPECT_EQ(cb->count, i);
}

void callOneThread(CallbackQueue* queue, bool& done)
{
  while (!done)
  {
    queue->callOne(ros::WallDuration(0.1));
  }
}

TEST(CallbackQueue, threadedCallOne)
{
  CountingCallbackPtr cb(boost::make_shared<CountingCallback>());
  size_t i = runThreadedTest(cb, callOneThread);
  ROS_INFO_STREAM(i);
  EXPECT_EQ(cb->count, i);
}

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

  for (uint32_t i = 0; i < 2; ++i)
  {
    tg.create_thread(boost::bind(callOneThread, &queue, boost::ref(done)));
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



