#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#define foreach BOOST_FOREACH

ros::AdvertiseOptions createAdvertiseOptions(const rosbag::ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    opts.latch = true;
    return opts;
}

struct CalleridFilter {
    CalleridFilter(const std::string& required_callerid) : required_callerid_(required_callerid) {}
    bool operator()(const rosbag::ConnectionInfo* conn) const {
        if (!conn->header) {
            return false;
        }   
        const ros::M_string& hs = *conn->header;
        ros::M_string::const_iterator latching_it = hs.find("latching");
        if (latching_it == hs.end() || latching_it->second != "1") {
            return false;
        }   

        ros::M_string::const_iterator callerid_it = hs.find("callerid");
        if (callerid_it == hs.end() || callerid_it->second.empty()) {
            return false;
        }   
        const std::string& callerid = callerid_it->second;
        return callerid == required_callerid_;
    }

    std::string required_callerid_;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "too few arguments" << std::endl;
        return 0;
    }

    std::string required_callerid(argv[1]);
    CalleridFilter filter(required_callerid);

    std::vector<boost::shared_ptr<rosbag::Bag> > bags;
    rosbag::View view;

    for (int i = 2; i < argc; ++i) {
        boost::shared_ptr<rosbag::Bag> bag(boost::make_shared<rosbag::Bag>());
        bag->open(argv[i], rosbag::bagmode::Read);
        bags.push_back(bag);
        view.addQuery(*bag, filter);
    }

    ros::init(argc, argv, required_callerid.substr(1, std::string::npos), ros::init_options::AnonymousName);
    ros::NodeHandle nh; 

    std::map<std::string, ros::Publisher> pubs;

    foreach(const rosbag::ConnectionInfo* c, view.getConnections()) {
        ros::AdvertiseOptions opts = createAdvertiseOptions(c, 1, "");
        pubs[c->topic] = nh.advertise(opts);
    }

    foreach(const rosbag::MessageInstance& m, view) {
        pubs.at(m.getTopic()).publish(m);
    }

    ros::spin();

    return 0;
}
