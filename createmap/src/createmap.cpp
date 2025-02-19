#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/bind.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

// Map parameter
const double GRID_RESOLUTION = 0.02;
const int MAP_SIZE_X = 400;
const int MAP_SIZE_Y = 400;
const int SCAN_THRESHOLD = 5;
const int DECAY_FACTOR = 1;

// Record the number of times the grid is scanned by the laser
std::vector<std::vector<int>> scan_count(MAP_SIZE_X, std::vector<int>(MAP_SIZE_Y, 0));

// Record whether it is confirmed as an obstacle
std::vector<std::vector<bool>> confirmed_obstacles(MAP_SIZE_X, std::vector<bool>(MAP_SIZE_Y, false));

// Record the number of times the grid is visited, if not, the value is -1
std::vector<std::vector<bool>> visited(MAP_SIZE_X, std::vector<bool>(MAP_SIZE_Y, false));

ros::Publisher map_publisher;
std::shared_ptr<tf::TransformListener> tf_listener;
std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;

bool map_origin_initialized = false;
tf::Vector3 map_origin_offset;

// center the map frame and odom frame
void broadcastMapFrame(const ros::Time& stamp)
{
    tf::Transform transform;
    if (map_origin_initialized)
    {
        transform.setOrigin(-map_origin_offset);
    } 
    else
    {
        transform.setOrigin(tf::Vector3(0, 0, 0));
    }
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster->sendTransform(tf::StampedTransform(transform, stamp, "map", "odom"));
}

// Using bresenhamLine to get the whole grid points on the line
std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> line;
    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy, e2;

    while (true)
    {
        line.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        { 
            err += dy; 
            x0 += sx; 
        }
        if (e2 <= dx)
        { 
            err += dx; 
            y0 += sy; 
        }
    }
    return line;
}

// World (x, y) -> Gridmap (x, y)
std::pair<int, int> worldToGrid(double x, double y)
{
    if (map_origin_initialized)
    {
        x += map_origin_offset.x();
        y += map_origin_offset.y();
    }
    int gx = static_cast<int>(x / GRID_RESOLUTION) + MAP_SIZE_X / 2;
    int gy = static_cast<int>(y / GRID_RESOLUTION) + MAP_SIZE_Y / 2;
    return {gx, gy};
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
    ros::Time scan_time = scan->header.stamp;
    try 
    {
        // align time
        tf::StampedTransform laser_transform;
        tf_listener->lookupTransform("map", scan->header.frame_id, scan_time, laser_transform);

        tf::StampedTransform base_transform;
        tf_listener->lookupTransform("map", "base_footprint", scan_time, base_transform);

        // initialize the map
        if (!map_origin_initialized)
        {
            map_origin_offset = base_transform.getOrigin();
            map_origin_initialized = true;
            ROS_INFO("Map origin initialized at robot starting position: (%.2f, %.2f)",
                     map_origin_offset.x(), map_origin_offset.y());
        }

        /*
        TODO:
        1. Set range threshold
        2. Get the start point and end point
        3. Check the status of each grid (bresenhamLine)
        */
        const double min_range_threshold = 0.1;  // 10cm
        double angle = scan->angle_min;

        // Traverse all laser measurements


            // map coordinates


            // For loop to check the status of grid
            // scan_count, confirmed_obstacles, visited

    } 
    catch (const tf::TransformException& ex) 
    {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }
}

// TODO: map -> odom
void publishMap(const ros::TimerEvent& event) 
{
    ros::Time current_time = ros::Time::now();
    broadcastMapFrame(current_time);


}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "create_gridmap");
    ros::NodeHandle nh;

    tf_listener = std::make_shared<tf::TransformListener>();
    tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 50, true);

    // Using message_filter is to align the timestamps of /scan and /tf
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 10);
    tf::MessageFilter<sensor_msgs::LaserScan> tf_filter(scan_sub, *tf_listener, "map", 10, nh);
    tf_filter.setTolerance(ros::Duration(0.1));
    tf_filter.registerCallback(boost::bind(&scanCallback, _1));

    // set a timer to publish the map
    ros::Timer map_timer = nh.createTimer(ros::Duration(0.05), publishMap);
    publishMap(ros::TimerEvent());
    ros::spin();
    return 0;
}

