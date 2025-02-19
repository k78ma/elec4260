#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_node");
    printf("This is a hello node!! \n");

    while(ros::ok())
    {
        printf("Hello world!\n");
    }
    return 0;
}