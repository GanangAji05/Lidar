#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher filtered_scan_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Define the minimum and maximum allowed infrared values
    float min_infrared = -100.0;  // Example minimum value
    float max_infrared = 10.5;  // Example maximum value

    // Create a new LaserScan message for the filtered data
    sensor_msgs::LaserScan filtered_scan_msg = *scan_msg;

    // Apply the filter to the infrared data
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        if (scan_msg->ranges[i] < min_infrared || scan_msg->ranges[i] > max_infrared) {
            // Set the infrared reading to NaN to remove it from the data
            filtered_scan_msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // Publish the filtered scan message
    filtered_scan_pub.publish(filtered_scan_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "infrared_filter_node");
    ros::NodeHandle nh;

    // Define the publisher for the filtered scan
    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("filter_IR", 1);

    // Subscribe to the raw laser scan topic
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    // Spin
    ros::spin();

    return 0;
}
