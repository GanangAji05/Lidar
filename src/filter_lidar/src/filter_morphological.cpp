#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered_cloud_pub;

void filteredPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& filtered_cloud_msg)
{
    // Convert the PointCloud2 message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*filtered_cloud_msg, *cloud);

    // Apply morphological dilation operation manually
    for (size_t i = 1; i < cloud->size() - 1; ++i) {
        pcl::PointXYZ& p = cloud->points[i];
        // Iterate over the neighboring points
        for (int dx = -100; dx <= 1; ++dx) {
            for (int dy = -100; dy <= 1; ++dy) {
                pcl::PointXYZ& neighbor = cloud->points[i + dx];
                if (neighbor.z > p.z) {
                    p.z = neighbor.z; // Update the z-coordinate to the maximum value among neighbors
                }
            }
        }
    }

    // Publish the filtered point cloud
    sensor_msgs::PointCloud2 filtered_cloud_out;
    pcl::toROSMsg(*cloud, filtered_cloud_out);
    filtered_cloud_out.header = filtered_cloud_msg->header;
    filtered_cloud_pub.publish(filtered_cloud_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_filter_node");
    ros::NodeHandle nh;

    // Subscribe to the filtered point cloud topic
    ros::Subscriber filtered_cloud_sub = nh.subscribe("/point_cloud", 1, filteredPointCloudCallback);

    // Create a publisher for the filtered point cloud
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_morph", 1);

    ros::spin();

    return 0;
}