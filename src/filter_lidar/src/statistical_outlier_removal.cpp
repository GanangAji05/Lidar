#include "ros/ros.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

ros::Publisher filtered_cloud_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
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

    // Apply RadiusOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> ptfilter;
    ptfilter.setInputCloud(cloud_in);
    ptfilter.setMeanK(4);  // Adjust the radius according to your needs
    ptfilter.setStddevMulThresh(1.0);  // Adjust the minimum number of neighbors according to your needs
    ptfilter.filter(*cloud_filtered);


    // Convert filtered PointCloud to PointCloud2
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);

    // Set the header information of the filtered PointCloud2 message
    filtered_cloud_msg.header = scan_msg->header;

    // Publish the filtered PointCloud2 message
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_statistical_removal");
    ros::NodeHandle nh;

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_statistical_removal", 1);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
