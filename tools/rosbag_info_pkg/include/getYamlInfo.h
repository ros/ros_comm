//
// Created by olga on 08.05.17.
//

#ifndef MYPAKCAGE_GETYAMLINFO_H
#define MYPAKCAGE_GETYAMLINFO_H


#include "RosbagInfo.h"
#include "yaml-cpp/yaml.h"

namespace rosbag{
    void printYamlInfo(const std::vector<std::string>& files,
                       const std::string& key, bool freq = false);
}





#endif //MYPAKCAGE_GETYAMLINFO_H
