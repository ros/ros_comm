#ifndef __SHM_TOPIC_HPP__
#define __SHM_TOPIC_HPP__

#include <boost/interprocess/managed_shared_memory.hpp>
#include "ros/ros.h"

#include "shm_publisher.hpp"
#include "shm_subscriber.hpp"

namespace shm_transport {

  class Topic {
    public:
      Topic(const ros::NodeHandle &);

      ~Topic();

      template < class M >
      Publisher advertise(const std::string & topic, uint32_t queue_size, uint32_t mem_size);

      template < class M >
      Subscriber< M > subscribe(const std::string & topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< const M > &));

    private:
      boost::shared_ptr< ros::NodeHandle > nh_;
  };

} // namespace shm_transport

#endif // __SHM_TOPIC_HPP__


