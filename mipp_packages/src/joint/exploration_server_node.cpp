#include <ExplorationServer.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting exploration_server_node.");

    ros::init(argc, argv, "exploration_server_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    ExplorationServer ExplorationServer(n, np);

    ros::spin();
    return 0;
}
