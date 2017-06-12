/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ROSBAG_BAGINFO_H
#define ROSBAG_BAGINFO_H

#include "rosbag/macros.h"

#include "rosbag/buffer.h"
#include "rosbag/chunked_file.h"
#include "rosbag/constants.h"
#include "rosbag/exceptions.h"
#include "rosbag/structures.h"

#include "ros/header.h"
#include "ros/time.h"
#include "ros/message_traits.h"
#include "ros/message_event.h"
#include "ros/serialization.h"




//#include "ros/subscription_callback_helper.h"

#include <ios>
#include <ostream>
#include <map>
#include <unordered_map>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <tuple>

#include <boost/format.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include "console_bridge/console.h"
#include "getYamlInfo.h"



namespace rosbag {
    //allows to read only those parts of file which are necessary for querying info
    enum ReadingMode {
        READ_CHUNK_INFO,
        READ_CHUNKS,
        READ_VERSION,
        READ_ALL
    };

    //returning value for key topics
    struct TopicInfo {
        std::string datatype;
        uint32_t connections;
        uint32_t msg_num;
        double frequency;
        TopicInfo():datatype("NONE"), connections(0), msg_num(0), frequency(0){}
    };



class MessageInstance;
class View;
class Query;



class  BagInfo
{
    friend class MessageInstance;
    friend class View;
public:

    explicit BagInfo(std::string const& filename, ReadingMode mode = READ_CHUNK_INFO);

    ~BagInfo();



    void close();

    uint32_t        getMajorVersion() const;
    uint32_t        getMinorVersion() const;
    uint64_t        getSize()         const;
    double       getStartTime()    const;
    double       getEndTime()      const;
    bool            isIndexed()       const;
    std::tuple<std::string, uint32_t, uint32_t>    getCompression()  const;





    std::map<std::string, TopicInfo > getTopics(bool freq) const;
    std::set<std::pair<std::string, std::string> > getTypes() const;
    uint64_t        getMessagesNumber() const;


private:


    void openRead  (std::string const& filename);





    void startReadingVersion102();
    void startReadingVersion200();



    void readVersion();
    void readFileHeaderRecord();
    void readConnectionRecord();
    void readChunkHeader(ChunkHeader& chunk_header) const;
    void readChunkInfoRecord();
    void readConnectionIndexRecord200();

    void readTopicIndexRecord102();
    void readMessageDefinitionRecord102();





    //void readHeaderFromBuffer(Buffer& buffer, uint32_t offset, ros::Header& header, uint32_t& data_size, uint32_t& bytes_read) const;
    bool readHeader(ros::Header& header) const;
    bool readDataLength(uint32_t& data_size) const;
    bool isOp(ros::M_string& fields, uint8_t reqOp) const;

    // Header fields





    template<typename T>
    bool readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data) const;

    bool readField(ros::M_string const& fields, std::string const& field_name, unsigned int min_len, unsigned int max_len, bool required, std::string& data) const;
    bool readField(ros::M_string const& fields, std::string const& field_name, bool required, std::string& data) const;

    bool readField(ros::M_string const& fields, std::string const& field_name, bool required, ros::Time& data) const;

    ros::M_string::const_iterator checkField(ros::M_string const& fields, std::string const& field,
                                             unsigned int min_len, unsigned int max_len, bool required) const;

    // Low-level I/O

    void read(char* b, std::streamsize n) const;
    void seek(uint64_t pos, int origin = std::ios_base::beg) const;

private:
    mutable ChunkedFile file_;
    int                 version_;
    CompressionType     compression_;
    uint32_t            chunk_threshold_;
    uint32_t            bag_revision_;


    uint64_t file_header_pos_;
    uint64_t index_data_pos_;
    uint32_t connection_count_;
    uint32_t chunk_count_;
    ReadingMode mode_;


    mutable std::map<std::string, uint32_t> compression_type_count_;

    // Current chunk
    bool      chunk_open_;
    ChunkInfo curr_chunk_info_;
    uint64_t  curr_chunk_data_pos_;

    std::map<std::string, uint32_t>                topic_connection_ids_;

    std::map<ros::M_string, uint32_t>              header_connection_ids_;
    std::map<uint32_t, ConnectionInfo*>            connections_;

    std::vector<ChunkInfo>                         chunks_;

    std::map<uint32_t, std::multiset<IndexEntry> > connection_indexes_;
    std::map<uint32_t, std::multiset<IndexEntry> > curr_chunk_connection_indexes_;


    mutable Buffer   header_buffer_;           //!< reusable buffer in which to assemble the record header before writing to file



    mutable Buffer*  current_buffer_;

    mutable uint64_t decompressed_chunk_;      //!< position of decompressed chunk
};

} // namespace rosbag

#include "rosbag/message_instance.h"

namespace rosbag {

// Templated method definitions



template<typename T>
bool BagInfo::readField(ros::M_string const& fields, std::string const& field_name, bool required, T* data) const {
    ros::M_string::const_iterator i = checkField(fields, field_name, sizeof(T), sizeof(T), required);
    if (i == fields.end())
    	return false;
    memcpy(data, i->second.data(), sizeof(T));
    return true;
}



} // namespace rosbag

#endif
