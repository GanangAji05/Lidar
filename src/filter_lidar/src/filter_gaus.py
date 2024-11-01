import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarFilter:
    def _init_(self):
        rospy.init_node('lidar_filter_node', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.filtered_scan_pub = rospy.Publisher('/filter_gaus', LaserScan, queue_size=10)

    def lidar_callback(self, data):
        ranges = data.ranges
        filtered_ranges = self.median_filter(ranges)
        filtered_scan = data
        filtered_scan.ranges = filtered_ranges
        self.filtered_scan_pub.publish(filtered_scan)

    def median_filter(self, data, window_size=5):
        filtered_data = []
        half_window = window_size // 2
        for i in range(len(data)):
            if i < half_window or i >= len(data) - half_window:
                filtered_data.append(data[i])
            else:
                window = data[i - half_window:i + half_window + 1]
                median_value = np.median(window)
                filtered_data.append(median_value)
        return filtered_data

    def run(self):
        rospy.spin()

if __name__ == '_main_':
    lidar_filter = LidarFilter()
    lidar_filter.run()