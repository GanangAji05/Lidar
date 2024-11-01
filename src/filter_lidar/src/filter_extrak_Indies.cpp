#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered_cloud_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Convert LaserScan to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

    // Assuming scan_msg contains points in polar coordinates
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // Convert polar coordinates to Cartesian coordinates
        pcl::PointXYZ point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;  // Assuming a 2D laser scan, set z to zero

        cloud_in->points.push_back(point);
    }

    // Create indices container
    pcl::PointIndices::Ptr indices_filtered(new pcl::PointIndices);

    // Apply PassThrough filter on the x-axis
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_in);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, 1000.0);
    pass_x.filter(indices_filtered->indices);

    // Apply PassThrough filter on the y-axis
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_in);
    pass_y.setIndices(indices_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(0.0, 5000.0);  // Adjust the limits according to your needs
    pass_y.filter(indices_filtered->indices);

    // Apply PassThrough filter on the z-axis
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_in);
    pass_z.setIndices(indices_filtered);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, 1000.0);
    pass_z.filter(indices_filtered->indices);

    // Apply ExtractIndices filter to get the final filtered point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices_filtered);
    extract.setNegative(false);  // Set to true to keep the points outside the specified indices
    extract.filter(*cloud_filtered);

    // Convert filtered PointCloud to PointCloud2
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);

    // Set the header information of the filtered PointCloud2 message
    filtered_cloud_msg.header = scan_msg->header;

    // Publish the filtered PointCloud2 message
    filtered_cloud_pub.publish(filtered_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_Com");
    ros::NodeHandle nh;

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud_Combine", 1);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
