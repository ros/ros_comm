#ifndef __SHM_SUBSCRIBER_HPP__
#define __SHM_SUBSCRIBER_HPP__

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/atomic/atomic.hpp>
#include "ros/ros.h"
#include "std_msgs/UInt64.h"

namespace shm_transport {

  class Topic;

  template <class M>
  class Subscriber;

  template < class M >
  class SubscriberCallbackHelper {
    typedef void (*Func)(const boost::shared_ptr< const M > &);

    private:
      SubscriberCallbackHelper(const std::string &, Func fp);

    public:
      ~SubscriberCallbackHelper();

      void callback(const std_msgs::UInt64::ConstPtr &);

    private:
      boost::interprocess::managed_shared_memory * pshm_;
      std::string topic_;
      Func fp_;
      boost::shared_ptr< ros::Subscriber > sub_;

    friend class Topic;
    friend class Subscriber<M>;
  };

  template <class M>
  class Subscriber {
    public:
      Subscriber(const Subscriber &);

    private:
      Subscriber(const ros::Subscriber &, SubscriberCallbackHelper< M > *);

    public:
      ~Subscriber();

      void shutdown();

      std::string getTopic() const;

      uint32_t getNumPublishers() const;

    protected:
      boost::shared_ptr< ros::Subscriber > sub_;
      SubscriberCallbackHelper< M > * phlp_;

    friend class Topic;
  };

} // namespace shm_transport

#endif // __SHM_SUBSCRIBER_HPP__

