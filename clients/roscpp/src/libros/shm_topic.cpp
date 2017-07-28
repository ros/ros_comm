#include <boost/atomic/atomic.hpp>
#include "ros/shm_topic.hpp"
#include "std_msgs/UInt64.h"

namespace shm_transport {

Topic::Topic(const ros::NodeHandle & parent) {
  nh_ = boost::make_shared<ros::NodeHandle>(parent);
}

Topic::~Topic() {
}

template < class M >
Publisher Topic::advertise(const std::string & topic, uint32_t queue_size, uint32_t mem_size) {
  ros::Publisher pub = nh_->advertise< std_msgs::UInt64 >(topic, queue_size);
  boost::interprocess::managed_shared_memory * pshm = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, topic.c_str(), mem_size);
  boost::atomic<uint32_t> *ref_ptr = pshm->find_or_construct<boost::atomic<uint32_t> >("ref")(0);
  ref_ptr->fetch_add(1, boost::memory_order_relaxed);
  return Publisher(pub, pshm);
}

template < class M >
Subscriber< M > Topic::subscribe(const std::string & topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< const M > &)) {
  SubscriberCallbackHelper< M > * phlp = new SubscriberCallbackHelper< M >(topic, fp);
  ros::Subscriber sub = nh_->subscribe(topic, queue_size, &SubscriberCallbackHelper< M >::callback, phlp);
  return Subscriber< M >(sub, phlp);
}

} // namespace shm_transport

