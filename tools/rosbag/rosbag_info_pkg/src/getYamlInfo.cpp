//
// Created by olga on 08.05.17.
//
#include <iostream>
#include <sstream>
#include "getYamlInfo.h"

using std::string;
using ros::Time;
using ros::Duration;
using std::stringstream;
using std::vector;

namespace rosbag{



    void printYamlInfo(const vector<std::string>& files,
                       const string& key, bool freq){
        YAML::Emitter info;

        for(const string& filename : files){
            if(key == "path"){
                BagInfo bag(filename, READ_VERSION);
                info << filename;
            }
            if(key == "version"){
                BagInfo bag(filename, READ_VERSION);
                info << std::to_string(bag.getMajorVersion()) + "."
                        + std::to_string(bag.getMinorVersion());
            }
            if(key == "size"){
                BagInfo bag(filename, READ_VERSION);
                info << bag.getSize();
            }
            if(key == "start"){
                BagInfo bag(filename);
                stringstream ss;
                ss  << std::fixed << std::setprecision(2) << bag.getStartTime();
                info << ss.str();
            }
            if(key == "end"){
                BagInfo bag(filename);
                stringstream ss;
                ss  << std::fixed << std::setprecision(2) << bag.getEndTime();
                info << ss.str();
            }
            if(key == "duration"){
                BagInfo bag(filename);
                stringstream ss;
                ss  << std::fixed << std::setprecision(6) << bag.getEndTime() - bag.getStartTime();
                info << ss.str();
            }
            if(key == "indexed"){
                BagInfo bag(filename);
                info << (bag.isIndexed() ? "True" : "False");
            }
            if(key == "messages"){
                BagInfo bag(filename);
                info << bag.getMessagesNumber();
            }
            if(key == "compression"){
                BagInfo bag(filename, READ_CHUNKS);
                auto compression_info = bag.getCompression();
                info << std::get<0>(compression_info);
                if(std::get<2>(compression_info)){
                    info << YAML::BeginMap;
                    info << YAML::Key << "uncompressed";
                    info << YAML::Value << std::get<1>(compression_info);
                    info << YAML::Key << "compressed";
                    info << YAML::Value << std::get<2>(compression_info);
                    info << YAML::EndMap;
                }
            }
            if(key == "topics"){
                BagInfo bag(filename, (freq ? READ_ALL : READ_CHUNK_INFO));
                auto topics = bag.getTopics(freq);
                info << YAML::BeginSeq;
                for(const auto& topic : topics){
                    info << YAML::BeginMap;
                    info << YAML::Key << "topic";
                    info << YAML::Value << topic.first;
                    if(topic.second.connections > 1){
                        info << YAML::Key << "connections";
                        info << YAML::Value << topic.second.connections;
                    }
                    info << YAML::Key << "messages";
                    info << YAML::Value << topic.second.msg_num;
                    if(freq && ! std::isinf(topic.second.frequency)) {
                        stringstream ss;
                        ss << std::fixed << std::setprecision(4) << topic.second.frequency;
                        //std::setprecision or precision method without std::fixed give wrong output
                        //getting rid of the trailing zeroes
                        string res = ss.str();
                        while(res.size() && res.back() == '0'){
                            res.resize(res.size() - 1);
                        }
                        info << YAML::Key << "frequency";
                        info << YAML::Value << res;
                    }
                    info << YAML::Key << "type";
                    info << YAML::Value << topic.second.datatype;
                    info << YAML::EndMap;
                }
                info << YAML::EndSeq;
            }
            if(key == "types"){
                BagInfo bag(filename);
                auto types = bag.getTypes();
                info << YAML::BeginSeq;
                for(const auto& type : types){
                    info << YAML::BeginMap;
                    info << YAML::Key << "type";
                    info << YAML::Value << type.first;
                    info << YAML::Key << "md5";
                    info << YAML::Value << type.second;
                    info << YAML::EndMap;
                }
                info << YAML::EndSeq;
            }
        }
        std::cout << info.c_str() << std::endl;
    }

}


