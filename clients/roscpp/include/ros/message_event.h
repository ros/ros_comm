
/*
 * Copyright (C) 2010, Willow Garage, Inc.
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

#ifndef ROSCPP_MESSAGE_EVENT_H
#define ROSCPP_MESSAGE_EVENT_H

#include "ros/forwards.h"
#include "ros/time.h"
#include <ros/assert.h>
#include <ros/message_traits.h>

#include <boost/type_traits/is_void.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>

namespace ros
{

template<typename M>
struct DefaultMessageCreator
{
  boost::shared_ptr<M> operator()()
  {
    return boost::make_shared<M>();
  }
};

template<typename M>
ROS_DEPRECATED inline boost::shared_ptr<M> defaultMessageCreateFunction()
{
  return DefaultMessageCreator<M>()();
}

/**
 * \brief Event type for subscriptions, const ros::MessageEvent<M const>& can be used in your callback instead of const boost::shared_ptr<M const>&
 *
 * Useful if you need to retrieve meta-data about the message, such as the full connection header, or the publisher's node name
 */
template<typename M>
class MessageEvent
{
public:
  typedef typename boost::add_const<M>::type ConstMessage;
  typedef typename boost::remove_const<M>::type Message;
  typedef boost::shared_ptr<Message> MessagePtr;
  typedef boost::shared_ptr<ConstMessage> ConstMessagePtr;
  typedef boost::function<MessagePtr()> CreateFunction;

  MessageEvent()
  : nonconst_need_copy_(true)
  {}

  MessageEvent(const MessageEvent<Message>& rhs)
  {
    *this = rhs;
  }

  MessageEvent(const MessageEvent<ConstMessage>& rhs)
  {
    *this = rhs;
  }

  MessageEvent(const MessageEvent<Message>& rhs, bool nonconst_need_copy)
  {
    *this = rhs;
    nonconst_need_copy_ = nonconst_need_copy;
  }

  MessageEvent(const MessageEvent<ConstMessage>& rhs, bool nonconst_need_copy)
  {
    *this = rhs;
    nonconst_need_copy_ = nonconst_need_copy;
  }

  MessageEvent(const MessageEvent<void const>& rhs, const CreateFunction& create)
  {
    init(boost::const_pointer_cast<Message>(boost::static_pointer_cast<ConstMessage>(rhs.getMessage())), rhs.getConnectionHeaderPtr(), rhs.getReceiptTime(), rhs.nonConstWillCopy(), create);
  }

  /**
   * \todo Make this explicit in ROS 2.0.  Keep as auto-converting for now to maintain backwards compatibility in some places (message_filters)
   */
  MessageEvent(const ConstMessagePtr& message)
  {
    init(message, getConnectionHeader(message.get()), ros::Time::now(), true, ros::DefaultMessageCreator<Message>());
  }

  MessageEvent(const ConstMessagePtr& message, const boost::shared_ptr<M_string>& connection_header, ros::Time receipt_time)
  {
    init(message, connection_header, receipt_time, true, ros::DefaultMessageCreator<Message>());
  }

  MessageEvent(const ConstMessagePtr& message, ros::Time receipt_time)
  {
    init(message, getConnectionHeader(message.get()), receipt_time, true, ros::DefaultMessageCreator<Message>());
  }

  MessageEvent(const ConstMessagePtr& message, const boost::shared_ptr<M_string>& connection_header, ros::Time receipt_time, bool nonconst_need_copy, const CreateFunction& create)
  {
    init(message, connection_header, receipt_time, nonconst_need_copy, create);
  }

  void init(const ConstMessagePtr& message, const boost::shared_ptr<M_string>& connection_header, ros::Time receipt_time, bool nonconst_need_copy, const CreateFunction& create)
  {
    message_ = message;
    connection_header_ = connection_header;
    receipt_time_ = receipt_time;
    nonconst_need_copy_ = nonconst_need_copy;
    create_ = create;
  }

  void operator=(const MessageEvent<Message>& rhs)
  {
    init(boost::static_pointer_cast<Message>(rhs.getMessage()), rhs.getConnectionHeaderPtr(), rhs.getReceiptTime(), rhs.nonConstWillCopy(), rhs.getMessageFactory());
    message_copy_.reset();
  }

  void operator=(const MessageEvent<ConstMessage>& rhs)
  {
    init(boost::const_pointer_cast<Message>(boost::static_pointer_cast<ConstMessage>(rhs.getMessage())), rhs.getConnectionHeaderPtr(), rhs.getReceiptTime(), rhs.nonConstWillCopy(), rhs.getMessageFactory());
    message_copy_.reset();
  }

  /**
   * \brief Retrieve the message.  If M is const, this returns a reference to it.  If M is non const
   * and this event requires it, returns a copy.  Note that it caches this copy for later use, so it will
   * only every make the copy once
   */
  boost::shared_ptr<M> getMessage() const
  {
    return copyMessageIfNecessary<M>();
  }

  /**
   * \brief Retrieve a const version of the message
   */
  const boost::shared_ptr<ConstMessage>& getConstMessage() const { return message_; }
  /**
   * \brief Retrieve the connection header
   */
  M_string& getConnectionHeader() const { return *connection_header_; }
  const boost::shared_ptr<M_string>& getConnectionHeaderPtr() const { return connection_header_; }

  /**
   * \brief Returns the name of the node which published this message
   */
  const std::string& getPublisherName() const { return connection_header_ ? (*connection_header_)["callerid"] : s_unknown_publisher_string_; }

  /**
   * \brief Returns the time at which this message was received
   */
  ros::Time getReceiptTime() const { return receipt_time_; }

  bool nonConstWillCopy() const { return nonconst_need_copy_; }
  bool getMessageWillCopy() const { return !boost::is_const<M>::value && nonconst_need_copy_; }

  bool operator<(const MessageEvent<M>& rhs)
  {
    if (message_ != rhs.message_)
    {
      return message_ < rhs.message_;
    }

    if (receipt_time_ != rhs.receipt_time_)
    {
      return receipt_time_ < rhs.receipt_time_;
    }

    return nonconst_need_copy_ < rhs.nonconst_need_copy_;
  }

  bool operator==(const MessageEvent<M>& rhs)
  {
    return message_ = rhs.message_ && receipt_time_ == rhs.receipt_time_ && nonconst_need_copy_ == rhs.nonconst_need_copy_;
  }

  bool operator!=(const MessageEvent<M>& rhs)
  {
    return !(*this == rhs);
  }

  const CreateFunction& getMessageFactory() const { return create_; }

private:
  template<typename M2>
  typename boost::disable_if<boost::is_void<M2>, boost::shared_ptr<M> >::type copyMessageIfNecessary() const
  {
    if (boost::is_const<M>::value || !nonconst_need_copy_)
    {
      return boost::const_pointer_cast<Message>(message_);
    }

    if (message_copy_)
    {
      return message_copy_;
    }

    ROS_ASSERT(create_);
    message_copy_ = create_();
    *message_copy_ = *message_;

    return message_copy_;
  }

  template<typename M2>
  typename boost::enable_if<boost::is_void<M2>, boost::shared_ptr<M> >::type copyMessageIfNecessary() const
  {
    return boost::const_pointer_cast<Message>(message_);
  }

  template<typename T>
  boost::shared_ptr<ros::M_string>
  getConnectionHeader(T* t, typename boost::enable_if<ros::message_traits::IsMessage<T> >::type*_ = 0) const
  {
    return t->__connection_header;
  }

  template<typename T>
  boost::shared_ptr<ros::M_string>
  getConnectionHeader(T* t, typename boost::disable_if<ros::message_traits::IsMessage<T> >::type*_ = 0) const
  {
    return boost::shared_ptr<ros::M_string>();
  }


  ConstMessagePtr message_;
  // Kind of ugly to make this mutable, but it means we can pass a const MessageEvent to a callback and not worry about other things being modified
  mutable MessagePtr message_copy_;
  boost::shared_ptr<M_string> connection_header_;
  ros::Time receipt_time_;
  bool nonconst_need_copy_;
  CreateFunction create_;

  static const std::string s_unknown_publisher_string_;
};

template<typename M> const std::string MessageEvent<M>::s_unknown_publisher_string_("unknown_publisher");


}

#endif // ROSCPP_MESSAGE_EVENT_H
