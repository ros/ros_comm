#include <boost/atomic/atomic.hpp>
#include "ros/shm_publisher.hpp"

namespace shm_transport {

Publisher::Publisher(const ros::Publisher & pub, boost::interprocess::managed_shared_memory * pshm) {
  pub_ = boost::make_shared< ros::Publisher >(pub);
  pshm_ = pshm;
}

Publisher::~Publisher() {
  boost::atomic<uint32_t> * ref_ptr = pshm_->find_or_construct<boost::atomic<uint32_t> >("ref")(0);
  if (ref_ptr->fetch_sub(1, boost::memory_order_relaxed) == 1) {
    boost::interprocess::shared_memory_object::remove(pub_->getTopic().c_str());
    ROS_INFO("shm file <%s>　removed\n", pub_->getTopic().c_str());
  }
  if (pshm_)
    delete pshm_;
  if (pub_)
    shutdown();
}

template < class M >
void Publisher::publish(const M & msg) const {
　　if (!pshm_)
  　　return;
　　if (pub_->getNumSubscribers() == 0)
  　　return;

　　uint32_t serlen = ros::serialization::serializationLength(msg);
　　uint32_t * ptr = (uint32_t *)pshm_->allocate(sizeof(uint32_t) * 2 + serlen);
　　ptr[0] = pub_->getNumSubscribers();
　　ptr[1] = serlen;
　　ros::serialization::OStream out((uint8_t *)(ptr + 2), serlen);
　　ros::serialization::serialize(out, msg);

　　std_msgs::UInt64 actual_msg;
　　actual_msg.data = pshm_->get_handle_from_address(ptr);
　　pub_->publish(actual_msg);
}

void Publisher::shutdown() {
  pub_->shutdown();
}

std::string Publisher::getTopic() const {
　　return pub_->getTopic();
}

uint32_t Publisher::getNumSubscribers() const {
  return pub_->getNumSubscribers();
}

} // namespace shm_transport

