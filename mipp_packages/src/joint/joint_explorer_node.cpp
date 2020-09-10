#include <JointExplorer.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting joint_explorer_node.");

    ros::init(argc, argv, "joint_explorer_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    JointExplorer JointExplorer(n, np);

    ros::spin();
    return 0;
}
