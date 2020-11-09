#include <MippPlanner.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting mipp_planner_node.");

    ros::init(argc, argv, "mipp_planner_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    MippPlanner MippPlanner(n, np);

    ros::spin();
    return 0;
}
