// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "RosbagInfo.h"



#if defined(_MSC_VER)
#include <stdint.h> // only on v2010 and later -> is this enough for msvc and linux?
#else
#include <inttypes.h>
#endif

#include <boost/foreach.hpp>

#include <algorithm>
#include <unordered_map>
#include <math.h>

#include "console_bridge/console.h"

#define foreach BOOST_FOREACH


using std::map;
using std::priority_queue;
using std::string;
using std::vector;
using std::multiset;
using boost::format;
using boost::shared_ptr;
using ros::M_string;
using ros::Time;
using std::tuple;
using std::pair;
using std::unordered_map;
using std::set;
using std::size_t;

namespace rosbag {



    BagInfo::BagInfo(string const& filename, ReadingMode mode) :
            compression_(compression::Uncompressed),
            chunk_threshold_(768 * 1024),  // 768KB chunks
            bag_revision_(0),
            file_header_pos_(0),
            index_data_pos_(0),
            connection_count_(0),
            chunk_count_(0),
            chunk_open_(false),
            curr_chunk_data_pos_(0),
            current_buffer_(0),
            decompressed_chunk_(0),
            mode_(mode)
    {
        openRead(filename);
    }

    BagInfo::~BagInfo() {
        close();
    }



    void BagInfo::openRead(string const& filename) {
        file_.openRead(filename);

        readVersion();
        if(mode_ != READ_VERSION || version_ != 200){
            switch (version_) {
                case 102: startReadingVersion102(); break;
                case 200: startReadingVersion200(); break;
                default:
                    throw BagException((format("Unsupported bag file version: %1%.%2%")
                                        % getMajorVersion() % getMinorVersion()).str());
            }
        }

    }


    void BagInfo::close() {
        if (!file_.isOpen())
            return;


        file_.close();

        topic_connection_ids_.clear();
        header_connection_ids_.clear();
        for (map<uint32_t, ConnectionInfo*>::iterator i = connections_.begin(); i != connections_.end(); i++)
            delete i->second;
        connections_.clear();
        chunks_.clear();
        connection_indexes_.clear();
        curr_chunk_connection_indexes_.clear();
    }





// Version


    void BagInfo::readVersion() {
        string version_line = file_.getline();

        file_header_pos_ = file_.getOffset();

        char logtypename[100];
        int version_major, version_minor;
#if defined(_MSC_VER)
        if (sscanf_s(version_line.c_str(), "#ROS%s V%d.%d", logtypename, sizeof(logtypename), &version_major, &version_minor) != 3)
#else
        if (sscanf(version_line.c_str(), "#ROS%s V%d.%d", logtypename, &version_major, &version_minor) != 3)
#endif
            throw BagIOException("Error reading version line");

        version_ = version_major * 100 + version_minor;

        logDebug("Read VERSION: version=%d", version_);
    }

    uint32_t BagInfo::getMajorVersion() const { return version_ / 100; }
    uint32_t BagInfo::getMinorVersion() const { return version_ % 100; }


    void BagInfo::startReadingVersion200() {
        // Read the file header record, which points to the end of the chunks
        readFileHeaderRecord();

        uint64_t chunk_pos = file_.getOffset();


        // Seek to the end of the chunks
        seek(index_data_pos_);

        // Read the connection records (one for each connection)
        for (uint32_t i = 0; i < connection_count_; i++)
            readConnectionRecord();

        // Read the chunk info records
        for (uint32_t i = 0; i < chunk_count_; i++)
            readChunkInfoRecord();

        if(mode_ == READ_ALL || mode_ == READ_CHUNKS){
            seek(chunk_pos);
            // Read the connection indexes for each chunk
                    foreach(ChunkInfo const& chunk_info, chunks_) {
                            curr_chunk_info_ = chunk_info;

                            //if the connection indexes records haven't been read
                            if(mode_ == READ_CHUNKS)
                                seek(curr_chunk_info_.pos);

                            // Skip over the chunk data
                            ChunkHeader chunk_header;
                            readChunkHeader(chunk_header);
                            seek(chunk_header.compressed_size, std::ios::cur);

                            // Read the index records after the chunk or skip them
                            if(mode_ == READ_ALL){
                                for (unsigned int i = 0; i < chunk_info.connection_counts.size(); i++)
                                    readConnectionIndexRecord200();
                            }

                        }

            // At this point we don't have a curr_chunk_info anymore so we reset it
            curr_chunk_info_ = ChunkInfo();
        }
    }

    void BagInfo::startReadingVersion102() {
        try
        {
            // Read the file header record, which points to the start of the topic indexes
            readFileHeaderRecord();
        }
        catch (BagFormatException ex) {
            throw BagUnindexedException();
        }

        // Get the length of the file
        seek(0, std::ios::end);
        uint64_t filelength = file_.getOffset();

        // Seek to the beginning of the topic index records
        seek(index_data_pos_);

        // Read the topic index records, which point to the offsets of each message in the file
        while (file_.getOffset() < filelength)
            readTopicIndexRecord102();

        // Read the message definition records (which are the first entry in the topic indexes)
        for (map<uint32_t, multiset<IndexEntry> >::const_iterator i = connection_indexes_.begin(); i != connection_indexes_.end(); i++) {
            multiset<IndexEntry> const& index       = i->second;
            IndexEntry const&           first_entry = *index.begin();

            logDebug("Reading message definition for connection %d at %llu", i->first, (unsigned long long) first_entry.chunk_pos);

            seek(first_entry.chunk_pos);

            readMessageDefinitionRecord102();
        }
    }

// File header record


    void BagInfo::readFileHeaderRecord() {
        ros::Header header;
        uint32_t data_size;
        if (!readHeader(header) || !readDataLength(data_size))
            throw BagFormatException("Error reading FILE_HEADER record");

        M_string& fields = *header.getValues();

        if (!isOp(fields, OP_FILE_HEADER))
            throw BagFormatException("Expected FILE_HEADER op not found");

        // Read index position
        readField(fields, INDEX_POS_FIELD_NAME, true, (uint64_t*) &index_data_pos_);

        if (index_data_pos_ == 0)
            throw BagUnindexedException();

        // Read topic and chunks count
        if (version_ >= 200) {
            readField(fields, CONNECTION_COUNT_FIELD_NAME, true, &connection_count_);
            readField(fields, CHUNK_COUNT_FIELD_NAME,      true, &chunk_count_);
        }

        logDebug("Read FILE_HEADER: index_pos=%llu connection_count=%d chunk_count=%d",
                 (unsigned long long) index_data_pos_, connection_count_, chunk_count_);

        // Skip the data section (just padding)
        seek(data_size, std::ios::cur);
    }



    void BagInfo::readChunkHeader(ChunkHeader& chunk_header) const {
        ros::Header header;
        if (!readHeader(header) || !readDataLength(chunk_header.compressed_size))
            throw BagFormatException("Error reading CHUNK record");

        M_string& fields = *header.getValues();

        if (!isOp(fields, OP_CHUNK))
            throw BagFormatException("Expected CHUNK op not found");

        readField(fields, COMPRESSION_FIELD_NAME, true, chunk_header.compression);
        readField(fields, SIZE_FIELD_NAME,        true, &chunk_header.uncompressed_size);

        compression_type_count_[chunk_header.compression]++;


        logDebug("Read CHUNK: compression=%s size=%d uncompressed=%d (%f)", chunk_header.compression.c_str(), chunk_header.compressed_size, chunk_header.uncompressed_size, 100 * ((double) chunk_header.compressed_size) / chunk_header.uncompressed_size);
    }

// Index records


    void BagInfo::readTopicIndexRecord102() {
        ros::Header header;
        uint32_t data_size;
        if (!readHeader(header) || !readDataLength(data_size))
            throw BagFormatException("Error reading INDEX_DATA header");
        M_string& fields = *header.getValues();

        if (!isOp(fields, OP_INDEX_DATA))
            throw BagFormatException("Expected INDEX_DATA record");

        uint32_t index_version;
        string topic;
        uint32_t count = 0;
        readField(fields, VER_FIELD_NAME,   true, &index_version);
        readField(fields, TOPIC_FIELD_NAME, true, topic);
        readField(fields, COUNT_FIELD_NAME, true, &count);

        logDebug("Read INDEX_DATA: ver=%d topic=%s count=%d", index_version, topic.c_str(), count);

        if (index_version != 0)
            throw BagFormatException((format("Unsupported INDEX_DATA version: %1%") % index_version).str());

        uint32_t connection_id;
        map<string, uint32_t>::const_iterator topic_conn_id_iter = topic_connection_ids_.find(topic);
        if (topic_conn_id_iter == topic_connection_ids_.end()) {
            connection_id = connections_.size();

            logDebug("Creating connection: id=%d topic=%s", connection_id, topic.c_str());
            ConnectionInfo* connection_info = new ConnectionInfo();
            connection_info->id       = connection_id;
            connection_info->topic    = topic;
            connections_[connection_id] = connection_info;

            topic_connection_ids_[topic] = connection_id;
        }
        else
            connection_id = topic_conn_id_iter->second;

        multiset<IndexEntry>& connection_index = connection_indexes_[connection_id];


        for (uint32_t i = 0; i < count; i++) {
            IndexEntry index_entry;
            uint32_t sec;
            uint32_t nsec;
            read((char*) &sec,                   4);
            read((char*) &nsec,                  4);
            read((char*) &index_entry.chunk_pos, 8);   //<! store position of the message in the chunk_pos field as it's 64 bits
            Time time(sec, nsec);
            index_entry.time = time;


            index_entry.offset = 0;

            logDebug("  - %d.%d: %llu", sec, nsec, (unsigned long long) index_entry.chunk_pos);

            if (index_entry.time < ros::TIME_MIN || index_entry.time > ros::TIME_MAX)
            {
                logError("Index entry for topic %s contains invalid time.", topic.c_str());
            } else
            {
                connection_index.insert(connection_index.end(), index_entry);
            }
        }
    }

    void BagInfo::readConnectionIndexRecord200() {
        ros::Header header;
        uint32_t data_size;
        if (!readHeader(header) || !readDataLength(data_size))
            throw BagFormatException("Error reading INDEX_DATA header");
        M_string& fields = *header.getValues();

        if (!isOp(fields, OP_INDEX_DATA))
            throw BagFormatException("Expected INDEX_DATA record");

        uint32_t index_version;
        uint32_t connection_id;
        uint32_t count = 0;
        readField(fields, VER_FIELD_NAME,        true, &index_version);
        readField(fields, CONNECTION_FIELD_NAME, true, &connection_id);
        readField(fields, COUNT_FIELD_NAME,      true, &count);

        logDebug("Read INDEX_DATA: ver=%d connection=%d count=%d", index_version, connection_id, count);

        if (index_version != 1)
            throw BagFormatException((format("Unsupported INDEX_DATA version: %1%") % index_version).str());

        uint64_t chunk_pos = curr_chunk_info_.pos;

        multiset<IndexEntry>& connection_index = connection_indexes_[connection_id];

        for (uint32_t i = 0; i < count; i++) {
            IndexEntry index_entry;
            index_entry.chunk_pos = chunk_pos;
            uint32_t sec;
            uint32_t nsec;
            read((char*) &sec,                4);
            read((char*) &nsec,               4);
            read((char*) &index_entry.offset, 4);
            index_entry.time = Time(sec, nsec);

            logDebug("  - %d.%d: %llu+%d", sec, nsec, (unsigned long long) index_entry.chunk_pos, index_entry.offset);

            if (index_entry.time < ros::TIME_MIN || index_entry.time > ros::TIME_MAX)
            {
                logError("Index entry for topic %s contains invalid time.  This message will not be loaded.", connections_[connection_id]->topic.c_str());
            } else
            {
                connection_index.insert(connection_index.end(), index_entry);
            }
        }
    }

// Connection records


    void BagInfo::readConnectionRecord() {
        ros::Header header;
        if (!readHeader(header))
            throw BagFormatException("Error reading CONNECTION header");
        M_string& fields = *header.getValues();

        if (!isOp(fields, OP_CONNECTION))
            throw BagFormatException("Expected CONNECTION op not found");

        uint32_t id;
        readField(fields, CONNECTION_FIELD_NAME, true, &id);
        string topic;
        readField(fields, TOPIC_FIELD_NAME,      true, topic);

        ros::Header connection_header;
        if (!readHeader(connection_header))
            throw BagFormatException("Error reading connection header");

        // If this is a new connection, update connections
        map<uint32_t, ConnectionInfo*>::iterator key = connections_.find(id);
        if (key == connections_.end()) {
            ConnectionInfo* connection_info = new ConnectionInfo();
            connection_info->id       = id;
            connection_info->topic    = topic;
            connection_info->header = boost::make_shared<M_string>();
            for (M_string::const_iterator i = connection_header.getValues()->begin(); i != connection_header.getValues()->end(); i++)
                (*connection_info->header)[i->first] = i->second;
            connection_info->msg_def  = (*connection_info->header)["message_definition"];
            connection_info->datatype = (*connection_info->header)["type"];
            connection_info->md5sum   = (*connection_info->header)["md5sum"];
            connections_[id] = connection_info;

            logDebug("Read CONNECTION: topic=%s id=%d", topic.c_str(), id);
        }
    }

    void BagInfo::readMessageDefinitionRecord102() {
        ros::Header header;
        uint32_t data_size;
        if (!readHeader(header) || !readDataLength(data_size))
            throw BagFormatException("Error reading message definition header");
        M_string& fields = *header.getValues();

        if (!isOp(fields, OP_MSG_DEF))
            throw BagFormatException("Expected MSG_DEF op not found");

        string topic, md5sum, datatype, message_definition;
        readField(fields, TOPIC_FIELD_NAME,               true, topic);
        readField(fields, MD5_FIELD_NAME,   32,       32, true, md5sum);
        readField(fields, TYPE_FIELD_NAME,                true, datatype);
        readField(fields, DEF_FIELD_NAME,    0, UINT_MAX, true, message_definition);

        ConnectionInfo* connection_info;

        map<string, uint32_t>::const_iterator topic_conn_id_iter = topic_connection_ids_.find(topic);
        if (topic_conn_id_iter == topic_connection_ids_.end()) {
            uint32_t id = connections_.size();

            logDebug("Creating connection: topic=%s md5sum=%s datatype=%s", topic.c_str(), md5sum.c_str(), datatype.c_str());
            connection_info = new ConnectionInfo();
            connection_info->id       = id;
            connection_info->topic    = topic;

            connections_[id] = connection_info;
            topic_connection_ids_[topic] = id;
        }
        else
            connection_info = connections_[topic_conn_id_iter->second];

        connection_info->msg_def  = message_definition;
        connection_info->datatype = datatype;
        connection_info->md5sum   = md5sum;
        connection_info->header = boost::make_shared<ros::M_string>();
        (*connection_info->header)["type"]               = connection_info->datatype;
        (*connection_info->header)["md5sum"]             = connection_info->md5sum;
        (*connection_info->header)["message_definition"] = connection_info->msg_def;

        logDebug("Read MSG_DEF: topic=%s md5sum=%s datatype=%s", topic.c_str(), md5sum.c_str(), datatype.c_str());
    }


    void BagInfo::readChunkInfoRecord() {
        // Read a CHUNK_INFO header
        ros::Header header;
        uint32_t data_size;
        if (!readHeader(header) || !readDataLength(data_size))
            throw BagFormatException("Error reading CHUNK_INFO record header");
        M_string& fields = *header.getValues();
        if (!isOp(fields, OP_CHUNK_INFO))
            throw BagFormatException("Expected CHUNK_INFO op not found");

        // Check that the chunk info version is current
        uint32_t chunk_info_version;
        readField(fields, VER_FIELD_NAME, true, &chunk_info_version);
        if (chunk_info_version != CHUNK_INFO_VERSION)
            throw BagFormatException((format("Expected CHUNK_INFO version %1%, read %2%") % CHUNK_INFO_VERSION % chunk_info_version).str());

        // Read the chunk position, timestamp, and topic count fields
        ChunkInfo chunk_info;
        readField(fields, CHUNK_POS_FIELD_NAME,  true, &chunk_info.pos);
        readField(fields, START_TIME_FIELD_NAME, true,  chunk_info.start_time);
        readField(fields, END_TIME_FIELD_NAME,   true,  chunk_info.end_time);
        uint32_t chunk_connection_count = 0;
        readField(fields, COUNT_FIELD_NAME,      true, &chunk_connection_count);

        logDebug("Read CHUNK_INFO: chunk_pos=%llu connection_count=%d start=%d.%d end=%d.%d",
                 (unsigned long long) chunk_info.pos, chunk_connection_count,
                 chunk_info.start_time.sec, chunk_info.start_time.nsec,
                 chunk_info.end_time.sec, chunk_info.end_time.nsec);


        // Read the topic count entries
        for (uint32_t i = 0; i < chunk_connection_count; i ++) {
            uint32_t connection_id, connection_count;
            read((char*) &connection_id,    4);
            read((char*) &connection_count, 4);

            logDebug("  %d: %d messages", connection_id, connection_count);

            chunk_info.connection_counts[connection_id] = connection_count;
        }

        chunks_.push_back(chunk_info);
    }

// Record I/O

    bool BagInfo::isOp(M_string& fields, uint8_t reqOp) const {
        uint8_t op = 0xFF; // nonexistent op
        readField(fields, OP_FIELD_NAME, true, &op);
        return op == reqOp;
    }


    bool BagInfo::readHeader(ros::Header& header) const {
        // Read the header length
        uint32_t header_len;
        read((char*) &header_len, 4);

        // Read the header
        header_buffer_.setSize(header_len);
        read((char*) header_buffer_.getData(), header_len);

        // Parse the header
        string error_msg;
        bool parsed = header.parse(header_buffer_.getData(), header_len, error_msg);
        if (!parsed)
            return false;

        return true;
    }

    bool BagInfo::readDataLength(uint32_t& data_size) const {
        read((char*) &data_size, 4);
        return true;
    }

    M_string::const_iterator BagInfo::checkField(M_string const& fields, string const& field, unsigned int min_len, unsigned int max_len, bool required) const {
        M_string::const_iterator fitr = fields.find(field);
        if (fitr == fields.end()) {
            if (required)
                throw BagFormatException("Required '" + field + "' field missing");
        }
        else if ((fitr->second.size() < min_len) || (fitr->second.size() > max_len))
            throw BagFormatException((format("Field '%1%' is wrong size (%2% bytes)") % field % (uint32_t) fitr->second.size()).str());

        return fitr;
    }

    bool BagInfo::readField(M_string const& fields, string const& field_name, bool required, string& data) const {
        return readField(fields, field_name, 1, UINT_MAX, required, data);
    }

    bool BagInfo::readField(M_string const& fields, string const& field_name, unsigned int min_len, unsigned int max_len, bool required, string& data) const {
        M_string::const_iterator fitr = checkField(fields, field_name, min_len, max_len, required);
        if (fitr == fields.end())
            return false;

        data = fitr->second;
        return true;
    }

    bool BagInfo::readField(M_string const& fields, string const& field_name, bool required, Time& data) const {
        uint64_t packed_time;
        if (!readField(fields, field_name, required, &packed_time))
            return false;

        uint64_t bitmask = (1LL << 33) - 1;
        data.sec  = (uint32_t) (packed_time & bitmask);
        data.nsec = (uint32_t) (packed_time >> 32);

        return true;
    }



// Low-level I/O



    void BagInfo::read(char* b, std::streamsize n) const  { file_.read(b, n);             }
    void BagInfo::seek(uint64_t pos, int origin) const    { file_.seek(pos, origin);      }

//My code starts here
    double to_sec(ros::Time time){
        return time.sec + time.nsec * 1e-9;
    }

    uint64_t BagInfo::getSize() const {
        uint64_t offset = file_.getOffset();
        seek(0, std::ios::end);
        uint64_t file_size = file_.getOffset();
        seek(offset);
        return file_size;
    }


    double BagInfo::getStartTime() const {
        Time start_time = ros::TIME_MAX;
        if (chunks_.size()) {
            start_time = chunks_[0].start_time;
        } else
            for (const auto &index : connection_indexes_) {
                Time time = (*index.second.begin()).time;
                if (time < start_time)
                    start_time = time;
            }
        return to_sec(start_time);
    }


    double BagInfo::getEndTime() const {
        Time end_time = ros::TIME_MIN;
        if (chunks_.size()) {
            end_time = chunks_.back().end_time;
        } else
            for (const auto &index : connection_indexes_) {
                auto last = --index.second.end();
                Time time = (*last).time;
                if (time < end_time)
                    end_time = time;
            }
        return to_sec(end_time);
    }

    double median(vector<double>& topic_periods){
        double result = 0;
        if(topic_periods.size() % 2 == 1){
            auto median_it = topic_periods.begin() + topic_periods.size() / 2;
            std::nth_element(topic_periods.begin(), median_it, topic_periods.end());
            result = *median_it;
        }
        else
        if(! topic_periods.empty()){
            auto lower_it = topic_periods.begin() + topic_periods.size() / 2 - 1;
            std::nth_element(topic_periods.begin(), lower_it, topic_periods.end());
            double lower = *lower_it;
            auto upper_it = topic_periods.begin() + topic_periods.size() / 2;
            std::nth_element(topic_periods.begin(), upper_it, topic_periods.end());
            double upper = *upper_it;
            result = (lower + upper) / 2;
        }
        return result;
    }

    map<string, TopicInfo >BagInfo::getTopics(bool freq) const {
        map<string, TopicInfo > topics;
        unordered_map<std::string, vector<double> > time_entries;
        for (const auto &connection : connections_) {
            ConnectionInfo *connection_info = connection.second;
            uint64_t msg_count = 0;

            for (const auto &chunk : chunks_) {
                auto it = chunk.connection_counts.find(connection_info->id);
                if (it != chunk.connection_counts.end())
                    msg_count += it->second;
            }
            auto& topic_in_map = topics[connection_info->topic];
            topic_in_map.msg_num += msg_count;
            topic_in_map.connections++;
            if(topic_in_map.datatype == "NONE")
                topic_in_map.datatype = connection_info->datatype;
            if(freq){
                auto& topic_time_entries = time_entries[connection_info->topic];
                auto it = connection_indexes_.find(connection_info->id);
                if(it != connection_indexes_.end()){
                    for(const IndexEntry& index : (*it).second)
                        topic_time_entries.push_back(to_sec(index.time));
                }
            }

        }
        if(freq) {
            for (auto &topic_time_entries : time_entries) {

                std::sort(topic_time_entries.second.begin(), topic_time_entries.second.end());
                vector<double> topic_periods;
                for (size_t i = 1; i < topic_time_entries.second.size(); i++) {
                    topic_periods.push_back(topic_time_entries.second[i] - topic_time_entries.second[i - 1]);
                }
                double median_val = median(topic_periods);
                topics[topic_time_entries.first].frequency = 1 / median_val;
            }
        }
        return topics;
    }



    uint64_t BagInfo::getMessagesNumber() const {
        uint64_t num_msg = 0;
        if(chunks_.size())
            for(const auto& chunk : chunks_)
                for(const auto& count : chunk.connection_counts)
                    num_msg += count.second;
        else
            for(const auto& index : connection_indexes_)
                num_msg += index.second.size();
        return num_msg;
    }


    bool BagInfo::isIndexed() const {
        return chunks_.size() || connection_indexes_.size();
    }


    tuple<string, uint32_t, uint32_t> BagInfo::getCompression() const {
        uint32_t main_compression_count = 0;
        string main_compression_type;
        uint32_t compressed = 0;
        uint32_t uncompressed = 0;
        for(const auto& compression_type : compression_type_count_){
            if(compression_type.second > main_compression_count){
                main_compression_count = compression_type.second;
                main_compression_type = compression_type.first;
            }
            if(compression_type.first == COMPRESSION_NONE)
                uncompressed += compression_type.second;
            else
                compressed += compression_type.second;
        }
        return std::make_tuple(main_compression_type, uncompressed, compressed);
    }


    set<pair<string, string> > BagInfo::getTypes() const {
        set<pair<string, string> > types;
        for(const auto& connection : connections_) {
            ConnectionInfo *connection_info = connection.second;
            types.insert({connection_info->datatype,
                          connection_info->md5sum});
        }
        return types;
    }











} // namespace rosbag

