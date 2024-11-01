#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

ros::Publisher filtered_cloud_pub;
laser_geometry::LaserProjection projector;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Convert LaserScan to PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    projector.projectLaser(*scan_msg, cloud_msg);

    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // Filter the PointCloud using CropBox
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(cloud);
    
    // Set CropBox filter parameters
    crop_filter.setMin(Eigen::Vector4f(-1.0, -0.1, -1.0, 1.0));
    crop_filter.setMax(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));

    // Apply CropBox filter
    crop_filter.filter(*filtered_cloud);

    // Publish the filtered PointCloud
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = scan_msg->header;
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_cropbox");
    ros::NodeHandle nh;

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filter_cropbox", 1);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
