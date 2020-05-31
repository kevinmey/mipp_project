/**
* @file rrt_planner_node.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Initializes an RRTPlanner instance
*/

#include <RRTPlanner.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting rrt_planner_node.");

    ros::init(argc, argv, "rrt_planner_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    RRTPlanner RRTPlanner(n, np);

    ros::spin();
    return 0;
}
