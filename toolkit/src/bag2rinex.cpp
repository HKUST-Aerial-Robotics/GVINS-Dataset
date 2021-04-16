#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>

#define INPUT_BAG_FILEPATH ""
#define OUTPUT_RINEX_FILEPATH ""

using namespace gnss_comm;

std::vector<std::vector<ObsPtr>> parse_gnss_meas(const std::string &bag_filepath)
{
    std::vector<std::vector<ObsPtr>> result;
    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/ublox_driver/range_meas");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m : view)
    {
        GnssMeasMsgConstPtr obs_msg = m.instantiate<GnssMeasMsg>();
        if (obs_msg == NULL)
            continue;
        std::vector<ObsPtr> obs = msg2meas(obs_msg);
        result.push_back(obs);
    }
    return result;
}

int main(int argc, char **argv)
{
    std::vector<std::vector<ObsPtr>> all_gnss_meas = parse_gnss_meas(INPUT_BAG_FILEPATH);
    obs2rinex(OUTPUT_RINEX_FILEPATH, all_gnss_meas);
    std::cout << "Done\n";
    return 0;
}