#include <UGVFrontierExplorer.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting ugv_frontier_explorer_node.");

    ros::init(argc, argv, "ugv_frontier_explorer_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    UGVFrontierExplorer UGVFrontierExplorer(n, np);

    ros::spin();
    return 0;
}
