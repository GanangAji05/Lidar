#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher filtered_cloud_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Convert laser scan to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        if (!std::isinf(scan_msg->ranges[i])) {  // Check for valid range values
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            pcl::PointXYZ point;
            point.x = scan_msg->ranges[i] * cos(angle);
            point.y = scan_msg->ranges[i] * sin(angle);
            point.z = 0.0;  // Assuming 2D laser scan, set z-coordinate to 0
            cloud->push_back(point);
        }
    }
    cloud->header.frame_id = scan_msg->header.frame_id;

    // Apply statistical outlier removal filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> AORI;
    AORI.setInputCloud(cloud);
    AORI.setMeanK(50);  // Set number of neighbors to consider
    AORI.setStddevMulThresh(1.0);  // Set threshold for outlier detection
    AORI.filter(*filtered_cloud);

    // Convert filtered point cloud to ROS message and publish
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = scan_msg->header;
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_to_cloud_filter");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe("/scan", 1, laserCallback);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filter_AORI", 1);

    ros::spin();

    return 0;
}