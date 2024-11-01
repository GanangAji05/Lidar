#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>

ros::Publisher filtered_scan_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Create a copy of the input LaserScan message
    sensor_msgs::LaserScan filtered_scan = *scan_msg;

    // Define Gaussian filter parameters
    double sigma = 0.1; // Adjust sigma as needed

    // Apply Gaussian filtering to the ranges
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        double weighted_sum = 0.0;
        double sum_weights = 0.0;
        for (int j = -1; j <= 1; ++j) { // Use a window size of 5
            int idx = i + j;
            if (idx >= 0 && idx < scan_msg->ranges.size()) {
                double weight = exp(-0.8 * pow(j / sigma, 2)); // Gaussian weight
                weighted_sum += weight * scan_msg->ranges[idx];
                sum_weights += weight;
            }
        }
        filtered_scan.ranges[i] = weighted_sum / sum_weights;
    }

    // Publish the filtered LaserScan message
    filtered_scan_pub.publish(filtered_scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gaussian_filter");
    ros::NodeHandle nh;

    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("filter_gaussian", 1);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    ros::spin();

    return 0;
}
