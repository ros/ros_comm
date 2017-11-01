#ifndef __SHM_PUBLISHER_HPP__
#define __SHM_PUBLISHER_HPP__

#include <boost/interprocess/managed_shared_memory.hpp>
#include "ros/ros.h"

namespace shm_transport {

  class Topic;

  class Publisher {
    private:
      Publisher(const ros::Publisher &, boost::interprocess::managed_shared_memory *);

    public:
      ~Publisher();

      template < class M >
      void publish(const M & topic) const;

      void shutdown();

      std::string getTopic() const;

      uint32_t getNumSubscribers() const;

    protected:
      boost::shared_ptr< ros::Publisher > pub_;
      boost::interprocess::managed_shared_memory * pshm_;

    friend class Topic;
  };

} // namespace shm_transport

#endif // __SHM_PUBLISHER_HPP__

