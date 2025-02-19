#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "romeo_node");
    printf("This is a Romeo! \n");

    ros::NodeHandle nh;
    ros::Publisher romeo_pub = nh.advertise<std_msgs::String>("romeo_topic", 2);

    ros::Rate loop_rate(2);

    while(ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Romeo is still alive!";
        romeo_pub.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}