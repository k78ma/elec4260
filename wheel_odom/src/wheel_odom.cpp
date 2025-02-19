#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h> 

// parameter of robot
const double WHEEL_RADIUS = 0.033; 
const double WHEEL_BASE   = 0.287; 
double last_left_position = 0.0;
double last_right_position = 0.0;

// Global pose（x, y, theta）
double x = 0.0;
double y = 0.0;
double theta = 0.0;

// Add this line
ros::Time last_time;

// Global publisher
ros::Publisher path_pub;    
ros::Publisher odom_pub;   
nav_msgs::Path path;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // In gazebo simulation
    double right_position = msg->position[0];
    double left_position  = msg->position[1];

    // In real-world robot experiment
    // double left_position = msg->position[0];
    // double right_position  = msg->position[1];

    // calculate the increment of two wheels
    double delta_left = left_position - last_left_position;
    double delta_right = right_position - last_right_position;

    // update position
    last_left_position = left_position;
    last_right_position = right_position;

    // calculate the displacement of two wheels
    double left_distance = delta_left * WHEEL_RADIUS;
    double right_distance = delta_right * WHEEL_RADIUS;

    // calculate the linear distance and rotation angle
    double linear_distance = (right_distance + left_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    // update the global pose
    theta += delta_theta;
    x += linear_distance * cos(theta);
    y += linear_distance * sin(theta);

    // publish the path
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "odom";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    // update the path
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    path_pub.publish(path);

    // publish the wheel odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    
    odom.twist.twist.linear.x = linear_distance / (msg->header.stamp - last_time).toSec();
    odom.twist.twist.angular.z = delta_theta / (msg->header.stamp - last_time).toSec();
    
    odom_pub.publish(odom);
    
    last_time = msg->header.stamp;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odometry_node");
    ros::NodeHandle nh;

    // path_pub and odom_pub
    path_pub = nh.advertise<nav_msgs::Path>("robot_path", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom", 10);

    // subscribe the joint states
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStateCallback);

    ros::spin();
    return 0;
}

