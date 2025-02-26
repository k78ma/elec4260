#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <memory>
#include <limits>

static double g_map_resolution = 0.02;
static int    g_map_width      = 0;
static int    g_map_height     = 0;

static double g_map_origin_x   = 0.0;
static double g_map_origin_y   = 0.0;
static double g_map_origin_z   = 0.0;

static pcl::PointCloud<pcl::PointXYZ>::Ptr gridmap_cloud(new pcl::PointCloud<pcl::PointXYZ>());

static bool g_has_map = false;

static std::shared_ptr<tf::TransformListener>    g_tf_listener;
static std::shared_ptr<tf::TransformBroadcaster>   g_tf_broadcaster;

static ros::Publisher g_icp_odom_pub;

static const double TF_DELAY = 0.05;

// obstacle to point cloud (value = 100) -> gridmap_cloud
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    g_map_width      = map_msg->info.width;
    g_map_height     = map_msg->info.height;
    g_map_resolution = map_msg->info.resolution;

    g_map_origin_x   = map_msg->info.origin.position.x;
    g_map_origin_y   = map_msg->info.origin.position.y;
    g_map_origin_z   = map_msg->info.origin.position.z;

    gridmap_cloud->clear();
    gridmap_cloud->header.frame_id = "map";

    for(int y = 0; y < g_map_height; ++y)
    {
        for(int x = 0; x < g_map_width; ++x)
        {
            int idx = y * g_map_width + x;
            int val = map_msg->data[idx];
            if(val == 100)
            { 
                double mx = g_map_origin_x + (x + 0.5) * g_map_resolution;
                double my = g_map_origin_y + (y + 0.5) * g_map_resolution;

                pcl::PointXYZ pt;
                pt.x = static_cast<float>(mx);
                pt.y = static_cast<float>(my);
                pt.z = 0.0f;
                gridmap_cloud->points.push_back(pt);
            }
        }
    }

    gridmap_cloud->width    = gridmap_cloud->points.size();
    gridmap_cloud->height   = 1;
    gridmap_cloud->is_dense = true;

    g_has_map = true;

    ROS_INFO("[mapCallback] Receive map: %d x %d, resolution=%.3f, obstacle pts=%zu",
             g_map_width, g_map_height, g_map_resolution, gridmap_cloud->points.size());
}

/*
Input : two set of point cloud
Output: Transformation matrix: Rotation matrix + transportation matrix
TODO:
1. compute the controid
2. H = sum((pi - c_src)*(qi - c_dst)^T)
3. SVD
4. R = V * U^T
5. t = centroid_dst - R * centroid_src
6. 3x3 matrix
*/
Eigen::Matrix3f computeBestRigidTransform2D(const std::vector<Eigen::Vector2f>& src,
                                             const std::vector<Eigen::Vector2f>& dst)
{
    // 1. compute the centroid
    Eigen::Vector2f centroid_src = Eigen::Vector2f::Zero();
    Eigen::Vector2f centroid_dst = Eigen::Vector2f::Zero();
    
    for (const auto& p : src) centroid_src += p;
    for (const auto& p : dst) centroid_dst += p;
    
    centroid_src /= static_cast<float>(src.size());
    centroid_dst /= static_cast<float>(dst.size());

    // 2. H = sum((pi - c_src)*(qi - c_dst)^T)
    Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        Eigen::Vector2f p_centered = src[i] - centroid_src;
        Eigen::Vector2f q_centered = dst[i] - centroid_dst;
        H += p_centered * q_centered.transpose();
    }

    // 3. SVD
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2f U = svd.matrixU();
    Eigen::Matrix2f V = svd.matrixV();

    // 4. R = V * U^T
    Eigen::Matrix2f R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(1) *= -1;
        R = V * U.transpose();
    }

    // 5. t = centroid_dst - R * centroid_src
    Eigen::Vector2f t = centroid_dst - R * centroid_src;

    // 6. 3x3 matrix
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
    T.block<2,2>(0,0) = R;
    T.block<2,1>(0,2) = t;
    
    return T;
}


/*
Input : two set of point cloud, dx, dy, dyaw
Output: update dx, dy, dyaw
TODO:
1. update the source point from last transformation
2. correspondence and nearest point search
3. Use computeBestRigidTransform2D to get the dT
*/
bool performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& mapCloud,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& currentCloud,
                double& dx, double& dy, double& dyaw)
{
    if (!mapCloud || mapCloud->empty() || !currentCloud || currentCloud->empty())
    {
        ROS_WARN("ICP input cloud is empty, skip. Map size: %zu, Current size: %zu", 
                 mapCloud ? mapCloud->size() : 0,
                 currentCloud ? currentCloud->size() : 0);
        return false;
    }

    // Add debug info
    ROS_INFO_THROTTLE(1.0, "Performing ICP with Map points: %zu, Current points: %zu", 
                      mapCloud->size(), currentCloud->size());

    // transfer to Eigen::Vector2f 
    std::vector<Eigen::Vector2f> targetPoints;
    targetPoints.reserve(mapCloud->size());
    for (const auto& pt : mapCloud->points)
    {
        targetPoints.emplace_back(pt.x, pt.y);
    }

    std::vector<Eigen::Vector2f> sourcePoints;
    sourcePoints.reserve(currentCloud->size());
    for (const auto& pt : currentCloud->points)
    {
        sourcePoints.emplace_back(pt.x, pt.y);
    }

    // init T
    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();

    // ICP parameter
    const int maxIterations = 20;
    const float converge_epsilon = 1e-4f;
    // TODO:
    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // 1. update the source point from last transformation
        std::vector<Eigen::Vector2f> transformedSource;
        transformedSource.reserve(sourcePoints.size());
        for (const auto& pt : sourcePoints) {
            Eigen::Vector3f p(pt.x(), pt.y(), 1.0f);
            Eigen::Vector3f transformed = T * p;
            transformedSource.emplace_back(transformed.x(), transformed.y());
        }

        // 2. correspondence and nearest point search
        std::vector<Eigen::Vector2f> srcMatched;
        std::vector<Eigen::Vector2f> dstMatched;
        
        for (const auto& p : transformedSource) {
            float minDist = std::numeric_limits<float>::max();
            Eigen::Vector2f closest;
            
            for (const auto& q : targetPoints) {
                float dist = (p - q).squaredNorm();
                if (dist < minDist) {
                    minDist = dist;
                    closest = q;
                }
            }
            
            // Only add if distance is within reasonable threshold
            if (minDist < 1.0) {  // 1 meter threshold
                srcMatched.push_back(p);
                dstMatched.push_back(closest);
            }
        }

        // If matched points is less than 3 pairs, then ICP fails
        if (srcMatched.size() < 3) {
            return false;
        }

        // 3. Use computeBestRigidTransform2D to get the dT
        Eigen::Matrix3f dT = computeBestRigidTransform2D(srcMatched, dstMatched);
        T = dT * T;

        // Check convergence
        float dx = dT(0,2);
        float dy = dT(1,2);
        float dr = std::atan2(dT(1,0), dT(0,0));
        
        if (std::abs(dx) < converge_epsilon && 
            std::abs(dy) < converge_epsilon && 
            std::abs(dr) < converge_epsilon) {
            break;
        }
    }

    // get dx dy dyaw from T
    dx = T(0,2);
    dy = T(1,2);
    dyaw = std::atan2(T(1,0), T(0,0));
    
    return true;
}


//TF: map->base_footprint
void broadcastMapToBaseFootprint(const ros::Time& stamp, double x, double y, double yaw)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);
    g_tf_broadcaster->sendTransform(tf::StampedTransform(transform, stamp, "map", "base_footprint"));
}

// TODO: Pub ICP odom
void publishIcpOdom(const ros::Time& stamp, double x, double y, double yaw)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_footprint";
    
    // Set position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    
    // Set orientation
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    // Publish the message
    g_icp_odom_pub.publish(odom);
}

// Project laser scan to point cloud in map frame
pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloudInMap(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::Time lookup_time = scan->header.stamp - ros::Duration(TF_DELAY);

    tf::StampedTransform tf_map_base;
    try
    {
        g_tf_listener->lookupTransform("map", "base_footprint", lookup_time, tf_map_base);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("Cannot lookup transform map->base_footprint: %s", ex.what());
        return nullptr;
    }

    tf::StampedTransform tf_base_laser;
    try
    {
        g_tf_listener->lookupTransform("base_footprint", scan->header.frame_id, lookup_time, tf_base_laser);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("Cannot lookup transform %s->base_footprint: %s", scan->header.frame_id.c_str(), ex.what());
        return nullptr;
    }

    double map_bf_x   = tf_map_base.getOrigin().x();
    double map_bf_y   = tf_map_base.getOrigin().y();
    double map_bf_yaw = tf::getYaw(tf_map_base.getRotation());

    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->header.frame_id = "map";

    double angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        double r = scan->ranges[i];
        if (std::isinf(r) || r < scan->range_min || r > scan->range_max)
        {
            angle += scan->angle_increment;
            continue;
        }
        // laser_frame 
        double lx = r * std::cos(angle);
        double ly = r * std::sin(angle);

        // base_footprint
        tf::Vector3 p_laser(lx, ly, 0.0);
        tf::Vector3 p_base = tf_base_laser * p_laser;

        // map->base_footprint
        double cos_yaw = std::cos(map_bf_yaw);
        double sin_yaw = std::sin(map_bf_yaw);
        double mx = map_bf_x + p_base.x() * cos_yaw - p_base.y() * sin_yaw;
        double my = map_bf_y + p_base.x() * sin_yaw + p_base.y() * cos_yaw;

        pcl::PointXYZ pt;
        pt.x = static_cast<float>(mx);
        pt.y = static_cast<float>(my);
        pt.z = 0.0f;
        cloud->points.push_back(pt);

        angle += scan->angle_increment;
    }

    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = true;

    return cloud;
}


// update map->base_footprint
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (!g_has_map)
    {
        ROS_WARN_THROTTLE(5.0, "No map yet, skip scan matching.");
        return;
    }

    ros::Time lookup_time = scan->header.stamp - ros::Duration(TF_DELAY);

    // Add debug info
    ROS_INFO_THROTTLE(1.0, "Received laser scan with %zu points", scan->ranges.size());

    // 1. project laser to map
    auto currentCloud = laserScanToPointCloudInMap(scan);
    if (!currentCloud)
    {
        ROS_WARN("Could not transform LaserScan to map frame, skip ICP.");
        return;
    }

    // 2. PerformICP
    double dx = 0.0, dy = 0.0, dyaw = 0.0;
    bool ok = performICP(gridmap_cloud, currentCloud, dx, dy, dyaw);
    if (!ok)
    {
        ROS_WARN("ICP did not converge, keep old map->base_footprint.");
        return;
    }

    // Add debug info for successful ICP
    ROS_INFO_THROTTLE(1.0, "ICP Success - dx: %.3f, dy: %.3f, dyaw: %.3f", dx, dy, dyaw);

    // 3. map->base_footprint
    tf::StampedTransform tf_map_base;
    try
    {
        g_tf_listener->lookupTransform("map", "base_footprint", lookup_time, tf_map_base);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("Cannot lookup transform map->base_footprint: %s", ex.what());
        return;
    }

    double old_x = tf_map_base.getOrigin().x();
    double old_y = tf_map_base.getOrigin().y();
    double old_yaw = tf::getYaw(tf_map_base.getRotation());

    // 4. Update the position and orientation
    double new_x   = old_x + dx;
    double new_y   = old_y + dy;
    double new_yaw = old_yaw + dyaw;

    // 5. update map->base_footprint
    broadcastMapToBaseFootprint(lookup_time, new_x, new_y, new_yaw);

    // 6. Pub ICP odom
    publishIcpOdom(lookup_time, new_x, new_y, new_yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_map_icp_odometry");
    ros::NodeHandle nh;

    // initialize the tf listener and broadcaster
    g_tf_listener = std::make_shared<tf::TransformListener>();
    g_tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserScanCallback);

    g_icp_odom_pub = nh.advertise<nav_msgs::Odometry>("/icp_odom", 10);

    ros::spin();
    return 0;
}
