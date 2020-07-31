/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 * Subscription queue test helper classes
 */
#ifndef TEST_ROSCPP_FAKE_MESSAGE_H
#define TEST_ROSCPP_FAKE_MESSAGE_H

#include "ros/subscription_callback_helper.h"

class FakeMessage
{
public:
  virtual const std::string __getDataType() const { return ""; }
  virtual const std::string __getMD5Sum() const { return ""; }
  virtual const std::string __getMessageDefinition() const { return ""; }
  virtual uint32_t serializationLength() const { return 0; }
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const { (void)seq; return write_ptr; }
  virtual uint8_t *deserialize(uint8_t *read_ptr) { return read_ptr; }
};

class FakeSubHelper : public ros::SubscriptionCallbackHelper
{
public:
  FakeSubHelper()
    : calls_(0)
  {}

  virtual ros::VoidConstPtr deserialize(const ros::SubscriptionCallbackHelperDeserializeParams&)
  {
    return boost::make_shared<FakeMessage>();
  }

  virtual std::string getMD5Sum() { return ""; }
  virtual std::string getDataType() { return ""; }

  virtual void call(ros::SubscriptionCallbackHelperCallParams& params)
  {
    (void)params;
    {
      boost::mutex::scoped_lock lock(mutex_);
      ++calls_;
    }

    if (cb_)
    {
      cb_();
    }
  }

  virtual const std::type_info& getTypeInfo() { return typeid(FakeMessage); }
  virtual bool isConst() { return true; }
  virtual bool hasHeader() { return false; }

  boost::mutex mutex_;
  uint32_t calls_;

  boost::function<void(void)> cb_;
};
typedef boost::shared_ptr<FakeSubHelper> FakeSubHelperPtr;

#endif // TEST_ROSCPP_FAKE_MESSAGE_H
