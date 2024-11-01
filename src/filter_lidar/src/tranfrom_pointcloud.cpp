#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class LaserScanToPointCloud {
public:
    LaserScanToPointCloud() : nh_("~") {
        scan_sub_ = nh_.subscribe("/scan", 1, &LaserScanToPointCloud::scanCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Populate point cloud from laser scan data
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            if (std::isinf(range)) {
                continue;  // Skip invalid range values
            }
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            pcl::PointXYZ point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0; // Assuming 2D laser scan
            cloud->push_back(point);
        }

        // Convert point cloud to ROS message and publish
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header = scan_msg->header;
        cloud_pub_.publish(cloud_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_to_point_cloud");
    LaserScanToPointCloud converter;
    ros::spin();
    return 0;
}