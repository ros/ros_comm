#include "ros/ros.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"
#include "rosbag/bag.h"
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <vector>
#include <string>

#define foreach BOOST_FOREACH


using std::vector;
using std::string;
using ros::Exception;
using rosbag::View;
using rosbag::Bag;
using rosbag::MessageInstance;
using rosbag::BagException;

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_by_node", ros::init_options::AnonymousName);
    try {
        Bag bag(argv[1]);
        View view(bag);
        foreach(MessageInstance instance, view) {
            if (instance.getCallerId() == argv[2]) {
                std::cout << instance.getTopic() << " " << instance.getTime().sec << " " << instance.getTime().nsec << std::endl;
            }
        }
    } catch(BagException& e) {
        std::cerr << "Error creating bag: " << e.what();
    } catch(Exception& e) {
        std::cerr << "Error: " << e.what();
    }
    return 0;
}
