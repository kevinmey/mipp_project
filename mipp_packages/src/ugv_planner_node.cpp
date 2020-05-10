/**
* @file ugv_planner_node.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Initializes an UGVPlanner instance
*/

#include <UGVPlanner.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting ugv_planner_node.");

    ros::init(argc, argv, "ugv_planner_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    //UGVPlanner UGVPlanner(n, np);

    ros::spin();
    return 0;
}
