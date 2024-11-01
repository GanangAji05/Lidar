#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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

    // Create user-defined indices (replace this with your logic)
    pcl::PointIndices::Ptr indices_in(new pcl::PointIndices);
    // Assuming you want to keep points at even indices
    for (size_t i = 0; i < cloud_in->size(); i += 2) {
        indices_in->indices.push_back(i);
    }

    // Create indices container
    pcl::PointIndices indices_out;

    // Apply ExtractIndices filter to get the final filtered point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices_in);
    extract.setNegative(false);  // Set to true to keep the points outside the specified indices
    extract.filter(*cloud_filtered);

    // Obtain the removed indices directly
    pcl::PointIndices indices_rem;
    extract.getRemovedIndices(indices_rem);

    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    extract.setNegative (false);
    extract.setUserFilterValue (1337.0);
    extract.filterDirectly (cloud_in);

    // Alternatively: the indices_out array is identical to indices_rem
    indices_out.indices = indices_rem.indices;

    // Convert filtered PointCloud to PointCloud2
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud_msg);

    // Set the header information of the filtered PointCloud2 message
    filtered_cloud_msg.header = scan_msg->header;

    // Publish the filtered PointCloud2 message
    filtered_cloud_pub.publish(filtered_cloud_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_Indices");
    ros::NodeHandle nh;

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("Indices", 1);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
