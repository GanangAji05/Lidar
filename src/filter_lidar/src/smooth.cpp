#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LidarSmoothingNode {
public:
    LidarSmoothingNode() {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to LiDAR data
        scan_sub_ = nh_.subscribe("/scan", 1, &LidarSmoothingNode::scanCallback, this);

        // Publish smoothed LiDAR data
        smooth_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/smoothed_scan", 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Apply moving average filter to LiDAR data
        sensor_msgs::LaserScan smooth_scan = *msg;
        for (size_t i = 1; i < msg->ranges.size() - 1; ++i) {
            smooth_scan.ranges[i] = (msg->ranges[i - 1] + msg->ranges[i] + msg->ranges[i + 1]) / 2.0;
        }

        // Publish smoothed LiDAR data
        smooth_scan_pub_.publish(smooth_scan);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher smooth_scan_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_smoothing_node");
    LidarSmoothingNode node;
    ros::spin();
    return 0;
}
