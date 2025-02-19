#include <ros/ros.h>
#include <std_msgs/String.h>

void romeoCallback(std_msgs::String msg)
{
    ROS_INFO(msg.data.c_str());
}

void julietCallback(std_msgs::String msg)
{
    ROS_WARN(msg.data.c_str());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "godfather_node");
    printf("This is the Godfather node! \n");

    ros::NodeHandle nh;
    ros::Subscriber godfather_sub = nh.subscribe("romeo_topic", 2, romeoCallback);
    ros::Subscriber godfather_sub1 = nh.subscribe("juliet_topic", 2, julietCallback);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}