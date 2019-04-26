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
 * random_access_bag.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */


#include "rosbag/random_access_bag.h"


using namespace std;


namespace rosbag {

/*
 * Only allows one single topic
 */
RandomAccessBag::RandomAccessBag(const rosbag::Bag &_bag,
                                 const std::string &topic) :

	rosbag::View::View(_bag, rosbag::TopicQuery(topic)),
	bagstore(_bag),
	viewTopic(topic)

{
  bagStartTime = getBeginTime();
  bagStopTime = getEndTime();

  createCache();
}

RandomAccessBag::~RandomAccessBag() {}

void RandomAccessBag::createCache() {
  update();
  rosbag::View::size();
  iterator it = begin();
  size_t sz = this->size();
  conn = getConnections()[0];
  msgPtr.resize(sz);

  for (uint32_t p = 0; p < sz; p++) {
    rosbag::MessageInstance &m = *it;
    rosbag::IndexEntry const ie = m.index_entry_;
    msgPtr.at(p) = ie;
    ++it;
  }
}

} // namespace rosbag
