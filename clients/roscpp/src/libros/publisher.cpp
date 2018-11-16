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

#include "ros/publisher.h"
#include "ros/publication.h"
#include "ros/node_handle.h"
#include "ros/subscriber_link.h"
#include "ros/topic_manager.h"

namespace ros
{

Publisher::Impl::Impl() : unadvertised_(false) { }

Publisher::Impl::~Impl()
{
  ROS_DEBUG("Publisher on '%s' deregistering callbacks.", topic_.c_str());
  unadvertise();
}

bool Publisher::Impl::isValid() const
{
  return !unadvertised_;
}

void Publisher::Impl::unadvertise()
{
  if (!unadvertised_)
  {
    unadvertised_ = true;
    TopicManager::instance()->unadvertise(topic_, callbacks_);
    node_handle_.reset();
  }
}

void Publisher::Impl::pushLastMessage(const SubscriberLinkPtr &sub_link) {
  boost::mutex::scoped_lock lock(last_message_mutex_);
  if (last_message_.buf) {
    sub_link->enqueueMessage(last_message_, true, true);
  }
}

Publisher::Publisher(const std::string& topic, const std::string& md5sum, const std::string& datatype, bool latch, const NodeHandle& node_handle, const SubscriberCallbacksPtr& callbacks)
: impl_(boost::make_shared<Impl>())
{
  impl_->topic_ = topic;
  impl_->md5sum_ = md5sum;
  impl_->datatype_ = datatype;
  impl_->latch_ = latch;
  impl_->node_handle_ = boost::make_shared<NodeHandle>(node_handle);
  impl_->callbacks_ = callbacks;
}

Publisher::Publisher(const Publisher& rhs)
{
  impl_ = rhs.impl_;
}

Publisher::~Publisher()
{
}

void Publisher::publish(const boost::function<SerializedMessage(void)>& serfunc, SerializedMessage& m) const
{
  if (!impl_)
  {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
    return;
  }

  if (!impl_->isValid())
  {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
    return;
  }

  TopicManager::instance()->publish(impl_->topic_, serfunc, m);

  if (isLatched()) {
    boost::mutex::scoped_lock lock(impl_->last_message_mutex_);
    impl_->last_message_ = m;
  }
}

void Publisher::incrementSequence() const
{
  if (impl_ && impl_->isValid())
  {
    TopicManager::instance()->incrementSequence(impl_->topic_);
  }
}

void Publisher::shutdown()
{
  if (impl_)
  {
    impl_->unadvertise();
    impl_.reset();
  }
}

std::string Publisher::getTopic() const
{
  if (impl_)
  {
    return impl_->topic_;
  }

  return std::string();
}

uint32_t Publisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid())
  {
    return TopicManager::instance()->getNumSubscribers(impl_->topic_);
  }

  return 0;
}

bool Publisher::isLatched() const {
  if (impl_ && impl_->isValid()) {
    return impl_->latch_;
  } else {
    ROS_ASSERT_MSG(false, "Call to isLatched() on an invalid Publisher");
    throw ros::Exception("Call to isLatched() on an invalid Publisher");
  }
}

} // namespace ros
