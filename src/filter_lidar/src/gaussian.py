#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import numpy as np
from scipy.ndimage import gaussian_filter

def lidarCallback(scan):
    # Convert LaserScan to numpy array
    ranges = np.array(scan.ranges)

    # Apply Gaussian filter
    sigma = 5.0  # Adjust sigma as needed
    filtered_ranges = gaussian_filter(ranges, sigma)

    # Publish filtered LaserScan data
    filtered_scan = scan
    filtered_scan.ranges = filtered_ranges.tolist()
    filtered_scan.header.stamp = rospy.Time.now()
    filtered_scan_pub.publish(filtered_scan)

def main():
    rospy.init_node('lidar_filter_node')
    rospy.Subscriber('/scan', LaserScan, lidarCallback)
    global filtered_scan_pub
    filtered_scan_pub = rospy.Publisher('/filtered_gaussian', LaserScan, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()