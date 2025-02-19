#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "juliet_node");
    printf("This is a Juliet! \n");

    ros::NodeHandle nh;
    ros::Publisher juliet_pub = nh.advertise<std_msgs::String>("juliet_topic", 2);

    ros::Rate loop_rate(2);

    while(ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Juliet is still alive!";
        juliet_pub.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}