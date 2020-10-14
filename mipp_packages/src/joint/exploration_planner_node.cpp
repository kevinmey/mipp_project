#include <ExplorationPlanner.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting exploration_planner_node.");

    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    ExplorationPlanner ExplorationPlanner(n, np);

    ros::spin();
    return 0;
}
