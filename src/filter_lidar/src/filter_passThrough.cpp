#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered_cloud_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Convert LaserScan to PointCloud2
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Assuming scan_msg contains points in polar coordinates
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // Convert polar coordinates to Cartesian coordinates
        pcl::PointXYZ point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;  // Assuming a 2D laser scan, set z to zero

        cloud->points.push_back(point);
    }

    // Convert PointCloud to PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);

    // Set the header information of the PointCloud2 message
    cloud_msg.header = scan_msg->header;

    // Filter parameters
    double min_range = -10.0;
    double max_range = 10000.0;
    std::string filter_axis = "z"; // Filter along the z-axis (height)

    // Apply PassThrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(filter_axis);
    pass.setFilterLimits(min_range, max_range);  // Adjust the limits according to your needs
    pass.filter(*cloud);

    // Convert filtered PointCloud to PointCloud2
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud, filtered_cloud_msg);

    // Set the header information of the filtered PointCloud2 message
    filtered_cloud_msg.header = scan_msg->header;

    // Publish the filtered PointCloud2 message
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_to_pointcloud_and_filter");
    ros::NodeHandle nh;

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
