#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

class LidarFilterNode {
public:
    LidarFilterNode() : nh("~") {
        // Subscribe to LiDAR data
        lidar_sub = nh.subscribe("/scan", 1, &LidarFilterNode::lidarCallback, this);

        // Advertise filtered LiDAR data
        filtered_lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/gaus", 1);
    }

    void lidarCallback(const sensor_msgs::LaserScanConstPtr& input_scan_msg) {
        // Apply Gaussian filter to range measurements
        std::vector<float> filtered_ranges = applyGaussianFilter(input_scan_msg->ranges);

        // Create filtered LaserScan message
        sensor_msgs::LaserScan filtered_scan_msg = *input_scan_msg;
        filtered_scan_msg.ranges = filtered_ranges;

        // Publish filtered data
        filtered_lidar_pub.publish(filtered_scan_msg);
    }

    std::vector<float> applyGaussianFilter(const std::vector<float>& ranges) {
        // Apply Gaussian filter to range measurements
        std::vector<float> filtered_ranges;
        for (size_t i = 0; i < ranges.size(); ++i) {
            float sum = 0.0;
            float weights_sum = 0.0;
            for (int j = -window_size_; j <= window_size_; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < ranges.size()) {
                    float weight = gaussianKernel(j, sigma_);
                    sum += ranges[idx] * weight;
                    weights_sum += weight;
                }
            }
            filtered_ranges.push_back(sum / weights_sum);
        }
        return filtered_ranges;
    }

    float gaussianKernel(int x, float sigma) {
        // Compute Gaussian kernel value
        return exp(-0.8 * (x * x) / (sigma * sigma)) / (sigma * sqrt(2 * M_PI));
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Publisher filtered_lidar_pub;

    const int window_size_ = 5; // Window size for Gaussian filter
    const float sigma_ = 0.01;   // Standard deviation for Gaussian filter
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gaus");
    LidarFilterNode lidar_filter_node;
    ros::spin();
    return 0;
}
