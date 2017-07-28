#include "ros/shm_subscriber.hpp"
#include "std_msgs/UInt64.h"

namespace shm_transport {

template < class M >
SubscriberCallbackHelper::SubscriberCallbackHelper(const std::string &topic, Func fp)
    : pshm_(NULL), topic_(topic), fp_(fp), sub_((ros::Subscriber *)NULL) {
}

template < class M >
SubscriberCallbackHelper::~SubscriberCallbackHelper() {
  boost::atomic<uint32_t> *ref_ptr = pshm_->find_or_construct<boost::atomic<uint32_t> >("ref")(0);
  if (ref_ptr->fetch_sub(1, boost::memory_order_relaxed) == 1) {
    boost::interprocess::shared_memory_object::remove(sub_->getTopic().c_str());
    ROS_INFO("shm file <%s> removed\n", sub_->getTopic().c_str());
  }
  if (pshm_)
    delete pshm_;
}

template < class M >
void SubscriberCallbackHelper::callback(const std_msgs::UInt64::ConstPtr & actual_msg) {
  if (!pshm_) {
    pshm_ = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only, topic_.c_str());
    boost::atomic<uint32_t> *ref_ptr = pshm_->find_or_construct<boost::atomic<uint32_t> >("ref")(0);
    ref_ptr->fetch_add(1, boost::memory_order_relaxed);
  }
  // FIXME this segment should be locked
  uint32_t * ptr = (uint32_t *)pshm_->get_address_from_handle(actual_msg->data);
  M msg;
  ros::serialization::IStream in((uint8_t *)(ptr + 2), ptr[1]);
  ros::serialization::deserialize(in, msg);
  // FIXME is boost::atomic rely on x86?
  if (reinterpret_cast< boost::atomic< uint32_t > * >(ptr)->fetch_sub(1, boost::memory_order_relaxed) == 1) {
    pshm_->deallocate(ptr);
  }
  fp_(boost::make_shared< M >(msg));
}


template <class M>
Subscriber::Subscriber(const Subscriber & s) {
  sub_ = s.sub_;
  phlp_ = s.phlp_;
}

template <class M>
Subscriber::Subscriber(const ros::Subscriber & sub, SubscriberCallbackHelper< M > * phlp) {
  sub_ = boost::make_shared< ros::Subscriber >(sub);
  phlp_ = phlp;
  phlp_->sub_ = sub_;
}

template <class M>
Subscriber::~Subscriber() {
  if (phlp_)
    delete phlp_;
  if (sub_)
    shutdown();
}

template <class M>
void Subscriber::shutdown() {
  sub_->shutdown();
}

template <class M>
std::string Subscriber::getTopic() const {
  return sub_->getTopic();
}

template <class M>
uint32_t Subscriber::getNumPublishers() const {
  return sub_->getNumPublishers();
}

} // namespace shm_transport

