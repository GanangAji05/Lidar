#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import numpy as np

def filter_lidar_data(scan_msg):
    # Convert the LIDAR data to a numpy array
    ranges = np.array(scan_msg.ranges)
    
    # Set up the Marker message to display the missing lines
    marker = Marker()
    marker.header.frame_id = scan_msg.header.frame_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.color.g = 1.0
    marker.color.a = 1.0
    
    # Find the missing lines and add them to the Marker message
    start_indices = np.where(ranges < scan_msg.range_max)[0]
    for i in range(len(start_indices) - 1):
        start_index = start_indices[i]
        end_index = start_indices[i + 1]
        if end_index - start_index > 1:
            missing_line = np.linspace(start_index, end_index, end_index - start_index)
            missing_ranges = ranges[missing_line.astype(int)]
            missing_points = np.zeros((len(missing_line), 3))
            missing_points[:, 0] = missing_line * scan_msg.angle_increment + scan_msg.angle_min
            missing_points[:, 1] = missing_ranges * np.cos(missing_points[:, 0])
            missing_points[:, 2] = missing_ranges * np.sin(missing_points[:, 0])
            for j in range(len(missing_points)):
                point = PointStamped()
                point.header.frame_id = scan_msg.header.frame_id
                point.point.x = missing_points[j, 0]
                point.point.y = missing_points[j, 1]
                point.point.z = missing_points[j, 2]
                marker.points.append(point)
            
    # Publish the filtered LIDAR data and the missing line Marker
    lidar_filtered_pub.publish(scan_msg)
    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('lidar_filter', anonymous=True)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, filter_lidar_data)
    lidar_filtered_pub = rospy.Publisher('/lidar_filtered', LaserScan, queue_size=10)
    marker_pub = rospy.Publisher('/missing_lines', Marker, queue_size=10)
    rospy.spin()