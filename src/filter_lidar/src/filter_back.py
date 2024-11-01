import rospy
from sensor_msgs.msg import LaserScan


def scan_callback(scan):
    filtered_ranges = []
    filtered_angles = []
    
    # Sudut yang ingin dihilangkan (misalnya, sinyal bagian belakang)
    min_angle =  -3.14159 # Sudut dalam radian
    max_angle = 3.14159  # Sudut dalam radian
    
    for i, angle in enumerate(scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))):
        if angle < min_angle or angle > max_angle:
            filtered_ranges.append(scan.ranges[i])
            filtered_angles.append(angle)

    filtered_scan = LaserScan()
    filtered_scan.header = scan.header
    filtered_scan.angle_min = min(filtered_angles)
    filtered_scan.angle_max = max(filtered_angles)
    filtered_scan.angle_increment = scan.angle_increment
    filtered_scan.time_increment = scan.time_increment
    filtered_scan.scan_time = scan.scan_time
    filtered_scan.range_min = scan.range_min
    filtered_scan.range_max = scan.range_max
    filtered_scan.ranges = filtered_ranges
    
    # Lakukan sesuatu dengan data yang sudah difilter (misalnya, publikasikan ke topik baru)
    pub_filtered_scan.publish(filtered_scan)

def listener():
    rospy.init_node('lidar_filter', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    global pub_filtered_scan
    pub_filtered_scan = rospy.Publisher('/lidar_filtered', LaserScan, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
