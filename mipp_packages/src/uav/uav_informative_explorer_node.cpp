#include <UAVInformativeExplorer.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting uav_server_node.");

    ros::init(argc, argv, "uav_server_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    UAVInformativeExplorer UAVInformativeExplorer(n, np);

    ros::spin();
    return 0;
}
