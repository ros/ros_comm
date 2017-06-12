#include <assert.h>
#include "getYamlInfo.h"


//note: this code only works with specified key and prints info in yaml format

using std::pair;
using std::string;
using std::vector;

struct Args {
    bool y = false;
    bool freq = false;
    string key;
    vector<string> files;
};

Args parse_arguments(int argc, char* argv[]) {
    Args res;
    int i = 1;
    while (i < argc) {
        if (std::string(argv[i]) == "-y")
            res.y = true;
        else
            if (std::string(argv[i]) == "-k" || std::string(argv[i]) == "--key")
                 res.key = std::string(argv[++i]);
            else
                if(std::string(argv[i]) == "--freq")
                    res.freq = true;
                else
                    res.files.push_back(string(argv[i]));
        i++;
    }
    //there is no exceptions here because it should be done in the original code as well as parsing arguments
    assert(res.y || res.key != "" || res.files.size());
    return res;
}



int main(int argc, char* argv[]){
    Args args = parse_arguments(argc, argv);
    try {
        rosbag::printYamlInfo(args.files, args.key, args.freq);
    }
    catch(const ros::Exception& e){
        std::cerr << e.what();
    }
    return 0;
}
