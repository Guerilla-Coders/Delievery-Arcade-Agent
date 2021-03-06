import rospy
import math
from sensor_msgs.msg import LaserScan
from standard_msgs.msg import Bool

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
WARNING_DISTANCE = 0.5

MAX_THRESHOLD = 3.5
MIN_THRESHOLD = 0

class ObstacleDetector():
    def __init__(self):
        self.obstacle_publisher = rospy.Publisher('obstacle_boolean', Bool, queue_size = 1)
        # self.rate = rospy.Rate(5)
        rospy.init_node('obstacle_warner')
        self.Bool = False

    def get_scan(self):
        """return a list that contains information of laser-scanned samples with index of each angles."""
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:  # samples_views will be classified as obstacles
            samples_view = samples

        if samples_view is 1:       # onlt 1 sample_views
            scan_filter.append(scan.ranges[0])

        else:                       # if sample view is lower than samples -> reset sample ranges and get info from the sensor
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = MAX_THRESHOLD
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = MIN_THRESHOLD
        
        
        return scan_filter

    def do_publish(self,boolean):
        self.obstacle_publisher.publish(boolean)
        # self.do_sleep()
    
    def do_sleep(self):
        self.rate.sleep()
    
    def obstacle(self):
        lidar_distances = self.get_scan()
        min_distance = min(lidar_distances)

        elif min_distance < WARNING_DISTANCE:
            self.Bool = True
        else:
            self.Bool = False
            
        self.do_publish(self.Bool)

    

def main():
    try:
        obstacle_warner = ObstacleDetector()
        obstacle_warner.obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()