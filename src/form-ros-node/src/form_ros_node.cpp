#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "form/form.hpp"
#include "form/utils.hpp"

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

static std::unique_ptr<form::Estimator> g_estimator;
static ros::Publisher g_odom_pub;
static ros::Publisher g_cloud_pub;
static std::string g_frame_id = "map";
static int g_num_rows    = 64;
static int g_num_columns = 1024;

// Convert sensor_msgs/PointCloud2 to a row-major scan vector expected by FORM.
// The output vector has num_rows * num_columns entries; invalid slots are NaN.
// Three strategies:
//   1. Organized + ring field  → use ring value as row index (best accuracy)
//   2. Organized, no ring      → use row = point_index / width
//   3. Unorganized             → bucket by elevation angle
static std::vector<form::PointXYZf>
cloud_to_form(const sensor_msgs::PointCloud2::ConstPtr& msg,
              int num_rows, int num_columns)
{
    const float kNaN = std::numeric_limits<float>::quiet_NaN();
    const int total = num_rows * num_columns;
    std::vector<form::PointXYZf> scan(total, form::PointXYZf(kNaN, kNaN, kNaN));

    bool has_ring = false;
    for (const auto& f : msg->fields) {
        if (f.name == "ring") { has_ring = true; break; }
    }

    const bool is_organized = (msg->height > 1);
    const uint32_t width    = msg->width;
    const size_t point_count = static_cast<size_t>(msg->width) * msg->height;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    if (has_ring) {
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
        for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_ring) {
            int ring = static_cast<int>(*iter_ring);
            int col  = static_cast<int>(i % width);
            if (ring < 0 || ring >= num_rows || col < 0 || col >= num_columns) continue;
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
            scan[ring * num_columns + col] = form::PointXYZf(*iter_x, *iter_y, *iter_z);
        }
    } else if (is_organized) {
        for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
            int row = static_cast<int>(i / width);
            int col = static_cast<int>(i % width);
            if (row >= num_rows || col >= num_columns) continue;
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) continue;
            scan[row * num_columns + col] = form::PointXYZf(*iter_x, *iter_y, *iter_z);
        }
    } else {
        // Unorganized: collect valid points, bucket by elevation angle
        struct RawPt { float x, y, z; };
        std::vector<RawPt> raw;
        raw.reserve(point_count);
        float elev_min =  std::numeric_limits<float>::max();
        float elev_max = -std::numeric_limits<float>::max();

        for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x, y = *iter_y, z = *iter_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            float r = std::sqrt(x*x + y*y);
            float elev = std::atan2(z, r);
            elev_min = std::min(elev_min, elev);
            elev_max = std::max(elev_max, elev);
            raw.push_back({x, y, z});
        }

        float elev_range = elev_max - elev_min + 1e-6f;
        std::vector<int> col_fill(num_rows, 0);
        for (auto& p : raw) {
            float r    = std::sqrt(p.x*p.x + p.y*p.y);
            float elev = std::atan2(p.z, r);
            int ring   = static_cast<int>((elev - elev_min) / elev_range * (num_rows - 1) + 0.5f);
            ring = std::max(0, std::min(ring, num_rows - 1));
            int col = col_fill[ring]++;
            if (col >= num_columns) continue;
            scan[ring * num_columns + col] = form::PointXYZf(p.x, p.y, p.z);
        }
    }

    return scan;
}

static geometry_msgs::Pose pose3_to_msg(const gtsam::Pose3& p)
{
    geometry_msgs::Pose msg;
    const auto& t = p.translation();
    msg.position.x = t.x();
    msg.position.y = t.y();
    msg.position.z = t.z();
    gtsam::Quaternion q = p.rotation().toQuaternion();
    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    return msg;
}

static void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // 1. Convert to FORM scan format
    auto scan = cloud_to_form(msg, g_num_rows, g_num_columns);

    // 2. Run FORM estimator
    g_estimator->register_scan(scan);

    // 3. Get current pose
    gtsam::Pose3 pose = g_estimator->current_lidar_estimate();

    // 4. Publish odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp    = msg->header.stamp;
    odom_msg.header.frame_id = g_frame_id;
    odom_msg.child_frame_id  = "lidar";
    odom_msg.pose.pose       = pose3_to_msg(pose);
    g_odom_pub.publish(odom_msg);

    // 5. Transform original scan into world frame and publish as registered cloud
    pcl::PointCloud<pcl::PointXYZ> registered_cloud;
    registered_cloud.reserve(scan.size());

    for (const auto& pt : scan) {
        if (!std::isfinite(pt.x)) continue;
        Eigen::Vector3d local(static_cast<double>(pt.x),
                              static_cast<double>(pt.y),
                              static_cast<double>(pt.z));
        Eigen::Vector3d world = pose * local;
        registered_cloud.push_back(
            pcl::PointXYZ(static_cast<float>(world.x()),
                          static_cast<float>(world.y()),
                          static_cast<float>(world.z())));
    }

    sensor_msgs::PointCloud2 registered_msg;
    pcl::toROSMsg(registered_cloud, registered_msg);
    registered_msg.header.stamp    = msg->header.stamp;
    registered_msg.header.frame_id = g_frame_id;
    g_cloud_pub.publish(registered_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "form_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string input_topic;
    pnh.param<std::string>("input_topic", input_topic, "/velodyne_points");
    pnh.param<int>("num_rows",    g_num_rows,    64);
    pnh.param<int>("num_columns", g_num_columns, 1024);

    // Configure and construct FORM estimator
    form::Estimator::Params params;
    params.extraction.num_rows    = static_cast<size_t>(g_num_rows);
    params.extraction.num_columns = static_cast<size_t>(g_num_columns);
    g_estimator = std::make_unique<form::Estimator>(params);

    g_odom_pub  = nh.advertise<nav_msgs::Odometry>("form/odometry", 10);
    g_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("form/registered_cloud", 10);

    ros::Subscriber sub = nh.subscribe(input_topic, 10, cloudCallback);

    ROS_INFO("FORM ROS node started. Input: %s  Geometry: %d rows x %d cols",
             input_topic.c_str(), g_num_rows, g_num_columns);

    ros::spin();
    return 0;
}
