/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * random_access_bag.h
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#ifndef _RANDOMACCESSBAG_H_
#define _RANDOMACCESSBAG_H_

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <utility>
#include <vector>

namespace rosbag {

/*
 * For simplicity, RandomAccessBag class is limited for single topic
 */
class RandomAccessBag : public rosbag::View {
public:

  RandomAccessBag(const rosbag::Bag &bag, const std::string &topic);

  virtual ~RandomAccessBag();

  template <typename T> boost::shared_ptr<T> at(uint64_t position) {
    assert(position >= 0 and position < size());
    return instantiate<T>(msgPtr.at(position));
  }

  size_t size() const { return static_cast<size_t>(size_cache_); }

  inline std::string topic() const { return viewTopic; }

  std::string messageType() const
  { return conn->datatype; }

protected:
  void createCache();

  const rosbag::Bag &bagstore;
  const rosbag::ConnectionInfo *conn;
  std::vector<rosbag::IndexEntry> msgPtr;

  template <class T>
  boost::shared_ptr<T> instantiate(const rosbag::IndexEntry &index_entry) {
    rosbag::MessageInstance *m =
        newMessageInstance(conn, index_entry, bagstore);
    return m->instantiate<T>();
  }

  ros::Time bagStartTime, bagStopTime;
  const std::string viewTopic;
};

} // namespace rosbag

#endif /* _RANDOMACCESSBAG_H_ */
