#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered_cloud_pub;
double leaf_size_x;
double leaf_size_y;
double leaf_size_z;
std::string filter_field_name;
double filter_limit_min;
double filter_limit_max;
bool filter_limit_negative;
std::string input_frame;
std::string output_frame;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Convert LaserScan to PointCloud2
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Assuming scan_msg contains points in polar coordinates
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // Convert polar coordinates to Cartesian coordinates
        pcl::PointXYZ point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;  // Assuming a 2D laser scan, set z to zero

        // Check for invalid (NaN or Inf) points
        if (!pcl::isFinite(point)) {
            continue;  // Skip invalid points
        }

        cloud_in->points.push_back(point);
    }

    // Check if the cloud is empty
    if (cloud_in->empty()) {
        ROS_WARN("Filtered point cloud is empty.");
        return;
    }

    // Convert PointCloud to PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_in, cloud_msg);

    // Set the header information of the PointCloud2 message
    cloud_msg.header = scan_msg->header;

    // Apply VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud_in);
    voxel_grid_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    voxel_grid_filter.setFilterFieldName(filter_field_name);
    voxel_grid_filter.setFilterLimits(filter_limit_min, filter_limit_max);
    voxel_grid_filter.setFilterLimitsNegative(filter_limit_negative);
    // voxel_grid_filter.setInputFrame(input_frame);
    // voxel_grid_filter.setOutputFrame(output_frame);
    voxel_grid_filter.filter(*cloud_filtered);

    // Convert filtered PointCloud to PointCloud2
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);

    // Set the header information of the filtered PointCloud2 message
    filtered_cloud_msg.header = scan_msg->header;

    // Publish the filtered PointCloud2 message
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_voxelgrid");
    ros::NodeHandle nh;

    nh.param("leaf_size", leaf_size_x, 0.1);
    nh.param("leaf_size", leaf_size_y, 0.01);
    nh.param("leaf_size", leaf_size_z, 0.01);
    nh.param("filter_field_name", filter_field_name, std::string("z"));
    nh.param("filter_limit_min", filter_limit_min, -10.0);
    nh.param("filter_limit_max", filter_limit_max, 12.0);
    nh.param("filter_limit_negative", filter_limit_negative, false);
    // nh.param("input_frame", input_frame, std::string("scan"));
    // nh.param("output_frame", output_frame, std::string("filter"));

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filter_voxelgrid", 1);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
