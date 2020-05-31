#include <UGVServer.hpp>

int main(int argc, char **argv)
{
    ROS_INFO("Starting ugv_server_node.");

    ros::init(argc, argv, "ugv_server_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    UGVServer UGVServer(n, np);

    ros::spin();
    return 0;
}
