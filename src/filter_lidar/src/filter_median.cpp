#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

ros::Publisher filtered_scan_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Create a copy of the input LaserScan message
    sensor_msgs::LaserScan filtered_scan = *scan_msg;

    // Apply median filtering to the ranges
    int window_size = 25;  // Adjust window size as needed
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        std::vector<float> window;
        int start_index = std::max(0, static_cast<int>(i) - window_size / 2);
        int end_index = std::min(static_cast<int>(scan_msg->ranges.size()) - 1, static_cast<int>(i) + window_size / 2);
        for (int j = start_index; j <= end_index; ++j) {
            window.push_back(scan_msg->ranges[j]);
        }
        std::sort(window.begin(), window.end());
        float median_value = window[window.size() / 2];
        filtered_scan.ranges[i] = median_value;
    }

    // Publish the filtered LaserScan message
    filtered_scan_pub.publish(filtered_scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "median_filter");
    ros::NodeHandle nh;

    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("filter_median", 1);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
